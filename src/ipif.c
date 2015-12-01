/*
 * Zynq-SJSU IP Interface: ipif.c
 *
 * This header file contains all the structures and function
 * prototypes of interface accessing via zynq-ipif driver.
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <ipif.h>

#define STRING_MAX	256

static int sysfs_read(char *node, u32 *value)
{
	char tmp[STRING_MAX];
	int fd, ret;

	if (!node)
		return -ENODEV;

	sprintf(tmp, "%s/%s", SYSFS_PATH, node);
	fd = open(tmp, O_RDONLY);
	ret = read(fd, tmp, STRING_MAX);
	sscanf(tmp, "%x", value);
	close(fd);

	if (ret == strlen(tmp) + 1)
		ret = sizeof(value);
	else
		ret = 0;

	return ret;
}

static int sysfs_write(char *node, u32 value)
{
	char tmp[STRING_MAX];
	int fd, ret;

	if (!node)
		return -ENODEV;

	sprintf(tmp, "%s/%s", SYSFS_PATH, node);
	fd = open(tmp, O_WRONLY);
	sprintf(tmp, "%x", value);
	ret = write(fd, tmp, STRING_MAX);
	close(fd);

	if (ret == strlen(tmp) + 1)
		ret = sizeof(value);
	else
		ret = 0;

	return ret;
}

static int reg_init(struct zynq_ipif_regmap *regmap, u32 size)
{
	u32 i;

	if (!regmap)
		return -ENODEV;

	for (i = 0; i < size; i++) {
		sysfs_write("reg_addr", regmap[i].addr);
		sysfs_write("reg_readable", regmap[i].readable_type);
		sysfs_write("reg_writable", regmap[i].writable_type);
		sysfs_write("reg_volatile", regmap[i].volatile_type);
		if (regmap[i].writable_type && regmap[i].val)
			sysfs_write("reg_val", regmap[i].val);
	}

	return 0;
}

int zynq_ipif_reg_read(struct zynq_ipif *ipif, u32 addr, u32 *val)
{
	struct zynq_ipif_regmap *regmap = ipif->regmap;
	int ret;

	sysfs_write("reg_addr", addr);
	ret = sysfs_read("reg_val", val);

	return ret == sizeof(val) ? 0 : -EINVAL;
}

int zynq_ipif_reg_write(struct zynq_ipif *ipif, u32 addr, u32 val)
{
	struct zynq_ipif_regmap *regmap = ipif->regmap;
	int ret;

	sysfs_write("reg_addr", addr);
	ret = sysfs_write("reg_val", val);

	return ret == sizeof(val) ? 0 : -EINVAL;
}

static int dma_default_condition(struct zynq_ipif_dma *dma)
{
	return 1;
}

static void *DMA_callback(void *threadid)
{
	struct zynq_ipif_dma *dma = (struct zynq_ipif_dma *)threadid;

	if (!dma->condition)
		dma->condition = dma_default_condition;

	while (dma->condition(dma)) {
		sem_wait(&dma->sem);
		if (dma->callback)
			dma->callback(dma);
	}

	pthread_exit(NULL);
}

void *DMA_engine_thread(void *threadid)
{
	struct zynq_ipif_dma_engine *dma_engine =
		(struct zynq_ipif_dma_engine *)threadid;
	struct epoll_event *events = dma_engine->events;
	struct zynq_ipif *parent = dma_engine->parent;
	struct zynq_ipif_dma *dma = parent->dma;
	int n, i, j, max = dma_engine->max_conn;

loop:
	n = epoll_wait(dma_engine->epfd, events, max, -1);
	if (n < 0 && errno == EINTR) {
		goto loop;
	} else if (n < 0) {
		printf("Epoll failed: %i\n", errno);
	} else if (n == 0) {
		printf("TIMEOUT\n");
	} else {
		for (i = 0; i < n; i++)
			for (j = 0; j < max; j++)
				if (events[i].data.fd == dma[j].fd)
					sem_post(&dma[j].sem);
	}

	goto loop;
}

int zynq_ipif_dma_init(struct zynq_ipif_dma *dma, struct zynq_ipif_dma_config *dma_config)
{
	u32 dma_size = dma_config->buf_size * dma_config->buf_num;
	struct zynq_ipif_dma_engine *dma_engine;
	char tmp[STRING_MAX];
	int flags, ret;

	if (!dma)
		return -ENODEV;

	dma_engine = &dma->parent->dma_engine;

	sprintf(tmp, "dma%d_cyclic", dma->index);
	sysfs_write(tmp, dma_config->cyclic);

	sprintf(tmp, "dma%d_width", dma->index);
	sysfs_write(tmp, dma_config->width);

	sprintf(tmp, "dma%d_burst", dma->index);
	sysfs_write(tmp, dma_config->burst);

	sprintf(tmp, "dma%d_buf_size", dma->index);
	sysfs_write(tmp, dma_config->buf_size);

	sprintf(tmp, "dma%d_buf_num", dma->index);
	sysfs_write(tmp, dma_config->buf_num);

	sprintf(tmp, "dma%d_%s", dma->index, dma_config->direction ? "src" : "dst");
	sysfs_write(tmp, dma_config->reg_addr);

	sprintf(tmp, "/dev/zynq_ipif_dma%d", dma->index);
	dma->fd = open(tmp, O_RDWR);
	if (!dma->fd)
		return -ENODEV;

	fcntl(dma->fd, F_SETOWN, getpid());
	flags = fcntl(dma->fd, F_GETFL);
	fcntl(dma->fd, F_SETFL, flags | FASYNC);

	sem_init(&dma->sem, 0, 0);

	ret = pthread_create(&dma->io_thread, NULL, DMA_callback, (void *)dma);
	if (ret) {
		printf("failed to create pthread: %d\n", ret);
		return ret;
	}

	dma->ev.events = EPOLLET;
	dma->ev.events |= !dma_config->direction ? EPOLLIN : EPOLLOUT;
	dma->ev.data.fd = dma->fd;

	ret = epoll_ctl(dma_engine->epfd, EPOLL_CTL_ADD, dma->fd, &dma->ev);
	if (ret < 0) {
		printf("Error epoll_ctl: %i\n", errno);
		return ret;
	}

	if (dma_config->access & DMA_BUF_ACCESS_TYPE_MASK == DMA_BUF_ACCESS_TYPE_MMAP) {
		if (!dma_config->cyclic) {
			printf("Scatter list does not work with mmap\n");
			return ret;
		}

		dma->buf = mmap(0, PAGE_ALIGN(dma_size),
				dma_config->direction ? PROT_READ : PROT_WRITE,
				MAP_SHARED, dma->fd, 0);
	}

	/* Copy them to prevent from being modified on the fly */
	dma->width = dma_config->width;
	dma->burst = dma_config->burst;
	dma->access = dma_config->access;
	dma->cyclic = dma_config->cyclic;
	dma->buf_num = dma_config->buf_num;
	dma->buf_size = dma_config->buf_size;
	dma->callback = dma_config->callback;
	dma->direction = dma_config->direction;
	dma->condition = dma_config->condition;

	dma->active = 1;
	/* TODO Error out restore */

	return 0;
}

int zynq_ipif_dma_read_buffer(struct zynq_ipif_dma *dma, u8 *buf, u32 size)
{
	if (!dma || !dma->active)
		return -ENODEV;

	dma->io_ptr += size / dma->width;

	return read(dma->fd, buf, size);
}

int zynq_ipif_dma_write_buffer(struct zynq_ipif_dma *dma, u8 *buf, u32 size)
{
	if (!dma || !dma->active)
		return -ENODEV;

	dma->io_ptr += size / dma->width;

	return write(dma->fd, buf, size);
}

int zynq_ipif_dma_enable(struct zynq_ipif_dma *dma, bool enable)
{
	char tmp[STRING_MAX];

	if (!dma || !dma->active)
		return -ENODEV;

	sprintf(tmp, "dma%d_ena", dma->index);
	sysfs_write(tmp, enable);

	return 0;
}

void zynq_ipif_dma_exit(struct zynq_ipif_dma *dma)
{
	if (!dma || !dma->active)
		return;

	if (dma)
		pthread_cancel(dma->io_thread);
	close(dma->fd);
}

void *IRQ_thread(void *threadid)
{
	struct zynq_ipif *ipif = (struct zynq_ipif *)threadid;
	struct epoll_event events;
	int n;

loop:
	n = epoll_wait(ipif->epfd, &events, 1, -1);
	if (n < 0 && errno == EINTR) {
		goto loop;
	} else if (n < 0) {
		printf("Epoll failed: %i\n", errno);
	} else if (n == 0) {
		printf("TIMEOUT\n");
	} else {
		if (events.data.fd == ipif->fd)
			sem_post(&ipif->sem);
	}

	goto loop;
}

void *IRQ_handler(void *threadid)
{
	struct zynq_ipif *ipif = (struct zynq_ipif *)threadid;

	while (ipif->irq_handler) {
		sem_wait(&ipif->sem);
		ipif->irq_handler(ipif);
	}

	pthread_exit(NULL);
}

int zynq_ipif_init(struct zynq_ipif *ipif, struct zynq_ipif_config *ipif_config)
{
	struct zynq_ipif_dma_engine *dma_engine = &ipif->dma_engine;
	struct epoll_event ev;
	int i, ret;

	if (!ipif) {
		printf("Null pointer for ipif\n");
		return -ENODEV;
	}

	memset(ipif, 0, sizeof(*ipif));

	if (!ipif_config->regmap) {
		printf("Null regmap\n");
		return -ENODEV;
	}

	ipif->regmap = ipif_config->regmap;
	ipif->irq_handler = ipif_config->irq_handler;

	for (i = 0; i < ARRAY_SIZE(ipif->dma); i++) {
		ipif->dma[i].index = i;
		ipif->dma[i].parent = ipif;
	}

	dma_engine->parent = ipif;
	dma_engine->max_conn = DMA_CHAN_MAX;
	dma_engine->epfd = epoll_create(DMA_CHAN_MAX);

	reg_init(ipif->regmap, ipif_config->regmap_size);

	ipif->fd = open("/dev/zynq_ipif_irq", O_RDWR);
	if (!ipif->fd)
		return -ENODEV;

	ipif->epfd = epoll_create1(0);

	ev.events = EPOLLIN | EPOLLET;
	ev.data.fd = ipif->fd;

	ret = epoll_ctl(ipif->epfd, EPOLL_CTL_ADD, ipif->fd, &ev);
	if (ret < 0)
		printf("Error epoll_ctl: %i\n", errno);

	ret = pthread_create(&ipif->epoll_thread, NULL, IRQ_thread, (void *)ipif);
	if (ret)
		printf("failed to create pthread: %d\n", ret);

	ret = pthread_create(&ipif->irq_thread, NULL, IRQ_handler, (void *)ipif);
	if (ret)
		printf("failed to create pthread: %d\n", ret);

	return 0;
}

int zynq_ipif_prepare_dma_engine(struct zynq_ipif *ipif)
{
	struct zynq_ipif_dma_engine *dma_engine = &ipif->dma_engine;
	int ret;

	ret = pthread_create(&dma_engine->thread, NULL, DMA_engine_thread, (void *)dma_engine);
	if (ret)
		printf("failed to create pthread: %d\n", ret);

	return ret;
}

int zynq_ipif_unprepare_dma_engine(struct zynq_ipif *ipif)
{
	struct zynq_ipif_dma_engine *dma_engine = &ipif->dma_engine;
	pthread_cancel(dma_engine->thread);
	return pthread_join(dma_engine->thread, NULL);
}

void zynq_ipif_exit(struct zynq_ipif *ipif)
{
	pthread_cancel(ipif->irq_thread);
	pthread_join(ipif->irq_thread, NULL);
	pthread_cancel(ipif->epoll_thread);
	pthread_join(ipif->epoll_thread, NULL);
	close(ipif->fd);
}
