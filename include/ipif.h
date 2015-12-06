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

/* May need to change it if someone change the address */
#define SYSFS_PATH	"/sys/bus/platform/drivers/zynq-ipif/43c00000.zynq_ipif"

/**
 * Register Map Configurations
 *
 * @addr: Relative address of a register
 * @val: Default value of the register
 * @readable_type: Readable attribute of the register
 * @writable_type: Writable attribute of the register
 * @volatile_type: Volatile attribute of the register
 */
struct zynq_ipif_regmap {
	u32 addr, val;
	u32 readable_type;
	u32 writable_type;
	u32 volatile_type;
};

/**
 * DMA Engine Private Data
 *
 * @events: Core epoll event to poll the DMA interrupt from kernel space
 * @parent: Pointer to zynq_ipif
 * @thread: Thread used to poll the DMA interrupt from kernel space
 * @max_conn: Maxium connection used by epoll
 * @epfd: File descriptor used by epoll
 * @init: Flag to prevent re-initialization
 */
struct zynq_ipif_dma_engine {
	struct epoll_event events[DMA_CHAN_MAX];
	struct zynq_ipif *parent;
	pthread_t thread;
	int max_conn;
	int epfd;
	bool init;
};

/**
 * DMA Channel Private Data
 *
 * @parent: Pointer to zynq_ipif
 * @ev: epoll event to poll the DMA interrupt from kernel space
 * @io_thread: Thread used to read or write DMA buffer
 * @buf_size: Size of each buffer in the ring buffer
 * @buf_num: Number of buffers in the ring buffer
 * @width: Data width of DMA transaction
 * @burst: Burst length of DMA transaction
 * @fd: File descriptor for DMA channel device
 * @index: DMA channel index (0~4)
 * @io_ptr: Pointer for read or write DMA buffer
 * @access: Acces mode of DMA buffer
 * @sem: Semphore used to synchronize DMA callback
 * @buf: Pointer of DMA buffer used by MMAP access mode
 * @direction: Direction of DMA transaction
 * @cyclic: DMA operation mode
 * @condition: Function to synchronously exit the DMA thread
 * @callback: User-definable DMA callback function
 */

struct zynq_ipif_dma {
	struct zynq_ipif *parent;
	struct epoll_event ev;
	pthread_t io_thread;
	u32 buf_size, buf_num;
	u32 burst, width;
	u32 fd, index;
	u32 io_ptr;
	u32 access;
	sem_t sem;
	void *buf;
	bool direction;
	bool active;
	bool cyclic;

	int (*callback) (struct zynq_ipif_dma *);
	int (*condition) (struct zynq_ipif_dma *);
};

/**
 * DMA Channel Configurations
 *
 * @reg_addr: Relative address of the data register
 * @buf_size: Size of each buffer in the ring buffer
 * @buf_num: Number of buffers in the ring buffer
 * @width: Data width of DMA transaction
 * @burst: Burst length of DMA transaction
 * @access: Acces mode of DMA buffer
 *	    0->DMA_BUF_ACCESS_TYPE_RWIO
 *	    1->DMA_BUF_ACCESS_TYPE_MMAP
 * @direction: Direction of DMA transaction
 *	    0->DMA_DIR_IN (DMA buffer to data register)
 *	    1->DMA_DIR_OUT (data register to DMA buffer)
 * @cyclic: DMA operation mode
 *	    0->one-shot mode (one interrupt for entire buffer)
 *	    1->cyclic mode (one interrupt for each buffer)
 * @condition: Function to synchronously exit the DMA thread
 * @callback: User-definable DMA callback function
 */
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

/**
 * IPIF Private Data
 *
 * @dma: DMA channel private data
 * @dma_engine: DMA engine private data
 * @epoll_thread: Thread used to poll interrupt from kernel space
 * @irq_thread: Thread of main interrupt sevice routine
 * @regmap: Register map private data
 * @mutex: Mutex lock to protect register access
 * @epfd: File descriptor used by epoll
 * @fd: File descriptor of IPIF device
 * @sem: Semphore used to synchronize interrupt handler
 * @irq_handler: User-definable interrupt handler
 */
struct zynq_ipif {
	struct zynq_ipif_dma dma[DMA_CHAN_MAX];
	struct zynq_ipif_dma_engine dma_engine;
	pthread_t epoll_thread, irq_thread;
	struct zynq_ipif_regmap *regmap;
	pthread_mutex_t mutex;
	int epfd, fd;
	sem_t sem;

	int (*irq_handler) (struct zynq_ipif *);
};

/**
 * IPIF Configurations
 *
 * @regmap: Pointer of register map configurations
 * @regmap_size: Size of register map
 * @irq_handler: User-definable interrupt handler
 */
struct zynq_ipif_config {
	struct zynq_ipif_regmap *regmap;
	u32 regmap_size;

	int (*irq_handler) (struct zynq_ipif *);
};

/* Initialize the library using correct configurations */
int zynq_ipif_init(struct zynq_ipif *, struct zynq_ipif_config *);
/* Load a shared epoll thread for DMA callbacks */
int zynq_ipif_prepare_dma_engine(struct zynq_ipif *);
/* Read registers */
int zynq_ipif_reg_read(struct zynq_ipif *, u32, u32 *);
/* Write registers */
int zynq_ipif_reg_write(struct zynq_ipif *, u32, u32);
/* Cancel the shared epoll thread */
int zynq_ipif_unprepare_dma_engine(struct zynq_ipif *);
/* Clean up and exit the library */
void zynq_ipif_exit(struct zynq_ipif *);

/* Initialize a DMA channel using correct configurations */
int zynq_ipif_dma_init(struct zynq_ipif_dma *, struct zynq_ipif_dma_config *);
/* Read DMA buffer of the DMA channel */
int zynq_ipif_dma_read_buffer(struct zynq_ipif_dma *, u8 *, u32);
/* Write DMA buffer of the DMA channel */
int zynq_ipif_dma_write_buffer(struct zynq_ipif_dma *, u8 *, u32);
/* Enable or disable DMA channel */
int zynq_ipif_dma_enable(struct zynq_ipif_dma *, bool);
/* Clean up the DMA channel */
void zynq_ipif_dma_exit(struct zynq_ipif_dma *);
