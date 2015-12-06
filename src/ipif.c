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

/* Read SYSFS nodes */
static int sysfs_read(char *node, u32 *value)
{
	char tmp[STRING_MAX];
	int fd, ret;

	if (!node)
		return -ENODEV;

	snprintf(tmp, STRING_MAX, "%s/%s", SYSFS_PATH, node);

	fd = open(tmp, O_RDONLY);
	if (fd < 0) {
		printf("failed to open %s: %d\n", tmp, errno);
		return -errno;
	}

	ret = read(fd, tmp, STRING_MAX);
	if (ret < 0) {
		printf("failed to read %s: %d\n", tmp, errno);
		close(fd);
		return -errno;
	}

	ret = sscanf(tmp, "%x", value);
	if (ret == EOF) {
		printf("failed to sscanf: %d\n", errno);
		close(fd);
		return -errno;
	}

	close(fd);

	return 0;
}

/* Write SYSFS nodes */
static int sysfs_write(char *node, u32 value)
{
	char tmp[STRING_MAX];
	int fd, ret;

	if (!node)
		return -ENODEV;

	snprintf(tmp, STRING_MAX, "%s/%s", SYSFS_PATH, node);

	fd = open(tmp, O_WRONLY);
	if (fd < 0) {
		printf("failed to open %s: %d\n", tmp, errno);
		return -errno;
	}

	snprintf(tmp, STRING_MAX, "%x", value);

	ret = write(fd, tmp, STRING_MAX);
	if (ret < 0) {
		printf("failed to write %s: %d\n", tmp, errno);
		close(fd);
		return -errno;
	}

	close(fd);

	return 0;
}

/* Initialize register map */
static int reg_init(struct zynq_ipif *ipif, u32 size)
{
	struct zynq_ipif_regmap *regmap = ipif->regmap;
	int ret, i;

	if (!regmap)
		return -ENODEV;

	pthread_mutex_lock(&ipif->mutex);
	for (i = 0; i < size; i++) {
		sysfs_write("reg_addr", regmap[i].addr);
		sysfs_write("reg_readable", regmap[i].readable_type);
		sysfs_write("reg_writable", regmap[i].writable_type);
		sysfs_write("reg_volatile", regmap[i].volatile_type);
		if (regmap[i].writable_type && regmap[i].val) {
			ret = sysfs_write("reg_val", regmap[i].val);
			if (ret) {
				printf("failed to write reg_val: %d\n", ret);
				pthread_mutex_unlock(&ipif->mutex);
				return ret;
			}
		}
	}
	pthread_mutex_unlock(&ipif->mutex);

	return 0;
}

int zynq_ipif_reg_read(struct zynq_ipif *ipif, u32 addr, u32 *val)
{
	struct zynq_ipif_regmap *regmap = ipif->regmap;
	int ret;

	pthread_mutex_lock(&ipif->mutex);
	sysfs_write("reg_addr", addr);
	ret = sysfs_read("reg_val", val);
	pthread_mutex_unlock(&ipif->mutex);
	if (ret)
		return -EINVAL;

	return ret;
}

int zynq_ipif_reg_write(struct zynq_ipif *ipif, u32 addr, u32 val)
{
	struct zynq_ipif_regmap *regmap = ipif->regmap;
	int ret;

	pthread_mutex_lock(&ipif->mutex);
	sysfs_write("reg_addr", addr);
	ret = sysfs_write("reg_val", val);
	pthread_mutex_unlock(&ipif->mutex);
	if (ret)
		return -EINVAL;

	return ret;
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

	/* Write DMA channel configurations */
	snprintf(tmp, STRING_MAX, "dma%d_cyclic", dma->index);
	ret = sysfs_write(tmp, dma_config->cyclic);
	if (ret)
		return ret;

	snprintf(tmp, STRING_MAX, "dma%d_width", dma->index);
	ret = sysfs_write(tmp, dma_config->width);
	if (ret)
		return ret;

	snprintf(tmp, STRING_MAX, "dma%d_burst", dma->index);
	ret = sysfs_write(tmp, dma_config->burst);
	if (ret)
		return ret;

	snprintf(tmp, STRING_MAX, "dma%d_buf_size", dma->index);
	ret = sysfs_write(tmp, dma_config->buf_size);
	if (ret)
		return ret;

	snprintf(tmp, STRING_MAX, "dma%d_buf_num", dma->index);
	ret = sysfs_write(tmp, dma_config->buf_num);
	if (ret)
		return ret;

	snprintf(tmp, STRING_MAX, "dma%d_%s", dma->index, dma_config->direction ? "src" : "dst");
	ret = sysfs_write(tmp, dma_config->reg_addr);
	if (ret)
		return ret;

	/* Open DMA channel device for callback polling */
	snprintf(tmp, STRING_MAX, "/dev/zynq_ipif_dma%d", dma->index);
	dma->fd = open(tmp, O_RDWR);
	if (dma->fd < 0)
		return -errno;

	/* Setup polling */
	ret = fcntl(dma->fd, F_SETOWN, getpid());
	if (ret < 0) {
		ret = -errno;
		goto fail_setown;
	}

	flags = fcntl(dma->fd, F_GETFL);
	if (flags < 0) {
		ret = -errno;
		goto fail_setown;
	}

	ret = fcntl(dma->fd, F_SETFL, flags | FASYNC);
	if (flags < 0) {
		ret = -errno;
		goto fail_setown;
	}

	sem_init(&dma->sem, 0, 0);

	ret = pthread_create(&dma->io_thread, NULL, DMA_callback, (void *)dma);
	if (ret) {
		printf("failed to create pthread: %d\n", ret);
		goto fail_setown;
	}

	dma->ev.events = EPOLLET;
	dma->ev.events |= !dma_config->direction ? EPOLLIN : EPOLLOUT;
	dma->ev.data.fd = dma->fd;

	/* Add this DMA channel to shared epoll of DMA engine */
	ret = epoll_ctl(dma_engine->epfd, EPOLL_CTL_ADD, dma->fd, &dma->ev);
	if (ret < 0) {
		printf("Error epoll_ctl: %i\n", errno);
		goto fail_epoll;
	}

	if (dma_config->access & DMA_BUF_ACCESS_TYPE_MASK == DMA_BUF_ACCESS_TYPE_MMAP) {
		if (!dma_config->cyclic) {
			printf("Scatter list does not work with mmap\n");
			ret = -EINVAL;
			goto fail_epoll;
		}

		/* Memory map the DMA buffer from kernel space to user space*/
		dma->buf = mmap(0, PAGE_ALIGN(dma_size),
				dma_config->direction ? PROT_READ : PROT_WRITE,
				MAP_SHARED, dma->fd, 0);
		if (dma->buf == MAP_FAILED) {
			printf("failed to execute memory map: %d\n", errno);
			ret = -errno;
			goto fail_epoll;
		}
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

	return 0;

fail_epoll:
	pthread_cancel(dma->io_thread);
fail_setown:
	close(dma->fd);

	return ret;
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
	int ret;

	if (!dma || !dma->active)
		return -ENODEV;

	/* Reset io pointer */
	dma->io_ptr = 0;

	snprintf(tmp, STRING_MAX, "dma%d_ena", dma->index);
	ret = sysfs_write(tmp, enable);
	if (ret) {
		printf("failed to %sable DMA channel %d\n",
		       enable ? "en" : "dis", dma->index);
		return -EINVAL;
	}

	return 0;
}

void zynq_ipif_dma_exit(struct zynq_ipif_dma *dma)
{
	if (!dma || !dma->active)
		return;

	if (dma->access & DMA_BUF_ACCESS_TYPE_MASK == DMA_BUF_ACCESS_TYPE_MMAP)
		munmap(dma->buf, PAGE_ALIGN(dma->buf_size * dma->buf_num));

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

	pthread_mutex_init(&ipif->mutex, NULL);

	ipif->regmap = ipif_config->regmap;
	ipif->irq_handler = ipif_config->irq_handler;

	for (i = 0; i < ARRAY_SIZE(ipif->dma); i++) {
		ipif->dma[i].index = i;
		ipif->dma[i].parent = ipif;
	}

	dma_engine->parent = ipif;
	dma_engine->max_conn = DMA_CHAN_MAX;
	dma_engine->epfd = epoll_create(DMA_CHAN_MAX);

	ret = reg_init(ipif, ipif_config->regmap_size);
	if (ret) {
		printf("failed to initialize regmap: %d\n", ret);
		return ret;
	}

	/* Open IPIF irq device */
	ipif->fd = open("/dev/zynq_ipif_irq", O_RDWR);
	if (ipif->fd < 0)
		return -errno;

	/* Use epoll to poll the irq from kernel space */
	ipif->epfd = epoll_create1(0);
	if (ipif->epfd < 0) {
		printf("failed to create epoll: %d\n", errno);
		goto fail_epoll;
	}

	ev.events = EPOLLIN | EPOLLET;
	ev.data.fd = ipif->fd;

	ret = epoll_ctl(ipif->epfd, EPOLL_CTL_ADD, ipif->fd, &ev);
	if (ret < 0) {
		printf("Error epoll_ctl: %i\n", errno);
		goto fail_epoll;
	}

	/* Thread for epoll */
	ret = pthread_create(&ipif->epoll_thread, NULL, IRQ_thread, (void *)ipif);
	if (ret) {
		printf("failed to create pthread: %d\n", ret);
		goto fail_epoll;
	}

	/* Thread for actual IRQ handling */
	ret = pthread_create(&ipif->irq_thread, NULL, IRQ_handler, (void *)ipif);
	if (ret) {
		printf("failed to create pthread: %d\n", ret);
		goto fail_irq_thread;
	}

	return 0;

fail_irq_thread:
	pthread_cancel(ipif->epoll_thread);
	pthread_join(ipif->epoll_thread, NULL);
fail_epoll:
	close(ipif->fd);

	return ret;
}

int zynq_ipif_prepare_dma_engine(struct zynq_ipif *ipif)
{
	struct zynq_ipif_dma_engine *dma_engine = &ipif->dma_engine;
	int ret;

	/* Create a shared thread for all DMA channels to poll DMA callbacks */
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
	pthread_mutex_destroy(&ipif->mutex);
}
