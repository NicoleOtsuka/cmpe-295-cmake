/*
 * Zynq-SJSU IP Interface Header Files: ipif.h
 *
 * This header file contains all the structures and function
 * prototypes of interface accessing via zynq-ipif driver.
 */

#include <pthread.h>
#include <semaphore.h>
#include <sys/epoll.h>
#include "utilis.h"

#define DMA_DIR_IN	0
#define DMA_DIR_OUT	1

#define DMA_CHAN_MAX	4

#define DEV_DMA_NAME	"zynq_ipif_dma"

#define DMA_BUF_ACCESS_TYPE_MASK	(1 << 0)
#define DMA_BUF_ACCESS_TYPE_RWIO	0
#define DMA_BUF_ACCESS_TYPE_MMAP	1

#define SYSFS_PATH	"/sys/bus/platform/drivers/zynq-ipif/43c00000.zynq_ipif"

struct zynq_ipif_regmap {
	u32 addr, val;
	u32 readable_type;
	u32 writable_type;
	u32 volatile_type;
};

struct zynq_ipif_dma_share {
	struct epoll_event events[DMA_CHAN_MAX];
	struct zynq_ipif *parent;
	pthread_t thread;
	int max_conn;
	int epfd;
	bool init;
};

struct zynq_ipif_dma {
	struct zynq_ipif *parent;
	struct epoll_event ev;
	pthread_t io_thread;
	u32 buf_size, buf_num;
	u32 io_ptr, buf_ptr;
	u32 burst, width;
	u32 fd, index;
	u32 access;
	sem_t sem;
	void *buf;
	bool direction;
	bool active;
	bool cyclic;

	int (*callback) (struct zynq_ipif_dma *);
	int (*condition) (struct zynq_ipif_dma *);
};

struct zynq_ipif_dma_config {
	u32 reg_addr;
	u32 buf_size;
	u32 buf_num;
	u32 width;
	u32 burst;
	u32 access;
	bool direction;
	bool cyclic;

	int (*condition) (struct zynq_ipif_dma *);
	int (*callback) (struct zynq_ipif_dma *);
};

struct zynq_ipif {
	struct zynq_ipif_dma dma[DMA_CHAN_MAX];
	struct zynq_ipif_dma_share dma_share;
	pthread_t epoll_thread, irq_thread;
	struct zynq_ipif_regmap *regmap;
	int epfd, fd;
	sem_t sem;

	int (*irq_handler) (struct zynq_ipif *);
};

struct zynq_ipif_config {
	struct zynq_ipif_regmap *regmap;
	u32 regmap_size;

	int (*irq_handler) (struct zynq_ipif *);
};

int reg_read(struct zynq_ipif_regmap *, u32, u32 *);
int reg_write(struct zynq_ipif_regmap *, u32, u32);

int dma_init(struct zynq_ipif_dma *, struct zynq_ipif_dma_config *);
int dma_read_buffer(struct zynq_ipif_dma *, u8 *, u32);
int dma_write_buffer(struct zynq_ipif_dma *, u8 *, u32);
int dma_enable(struct zynq_ipif_dma *, bool);
void dma_exit(struct zynq_ipif_dma *);

int zynq_ipif_init(struct zynq_ipif *, struct zynq_ipif_config *);
int zynq_ipif_prepare_dma_share(struct zynq_ipif_dma_share *);
int zynq_ipif_unprepare_dma_share(struct zynq_ipif_dma_share *);
void zynq_ipif_exit(struct zynq_ipif *);
