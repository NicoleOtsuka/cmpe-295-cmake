#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <ipif.h>

#define DATA_WIDTH	0x4
#define DATA_BURST	0x4

#define PERIOD_NUM	0x2
#define PERIOD_SIZE	(DATA_BURST * DATA_WIDTH)
#define BUF_SIZE	(PERIOD_SIZE * PERIOD_NUM)

static struct zynq_ipif_regmap regmap[] = {
	/* Addr value   read	write	volatile */
	{0x0,	0,	1,	1,	0},	/* REG_CTRL_RESET */
	{0x4,	0,	1,	1,	0},	/* REG_CTRL_ENABLE */
	{0x8,	0,	1,	1,	0},	/* REG_CTRL_IRQMSK */
	{0xc,	1,	1,	1,	0},	/* REG_CTRL_DMA0_WM */
	{0x10,	0,	1,	1,	0},	/* REG_CTRL_DMA1_WM */
	{0x14,	1,	1,	1,	0},	/* REG_CTRL_DMA2_WM */
	{0x18,	0,	1,	1,	0},	/* REG_CTRL_DMA3_WM */
	{0x1c,	0,	1,	1,	0},	/* REG_CTRL_IP_DELAY */

	{0x100,	0,	1,	0,	1},	/* REG_STAT_IRQ */
	{0x104,	0,	1,	0,	1},	/* REG_STAT_FIFO0_CNT */
	{0x108,	0,	1,	0,	1},	/* REG_STAT_FIFO1_CNT */
	{0x10c,	0,	1,	0,	1},	/* REG_STAT_FIFO2_CNT */
	{0x110,	0,	1,	0,	1},	/* REG_STAT_FIFO3_CNT */

	/* 0x200 - Leave all data registers to DMA */

	{0x300,	0,	1,	0,	1},	/* REG_DEBG_DMA_SIG */
	{0x304,	0,	1,	0,	1},	/* REG_DEBG_DMA_STAT0 */
	{0x308,	0,	1,	0,	1},	/* REG_DEBG_DMA_STAT1 */
	{0x30c,	0,	1,	0,	1},	/* REG_DEBG_DMA_STAT2 */
	{0x310,	0,	1,	0,	1},	/* REG_DEBG_DMA_STAT3 */
	{0x314,	0,	1,	0,	1},	/* REG_DEBG_DMA_ACKCNT0 */
	{0x318,	0,	1,	0,	1},	/* REG_DEBG_DMA_ACKCNT1 */
	{0x31c,	0,	1,	0,	1},	/* REG_DEBG_DMA_ACKCNT2 */
	{0x320,	0,	1,	0,	1},	/* REG_DEBG_DMA_ACKCNT3 */
	{0x324,	0,	1,	0,	1},	/* REG_DEBG_IRQ_SIG */
};

static int IRQ_handler(struct zynq_ipif *ipif)
{
	printf("---%s---\n", __func__);
	return 0;
}

static struct zynq_ipif_config ipif_config = {
	.regmap = regmap,
	.regmap_size = ARRAY_SIZE(regmap),
	.irq_handler = IRQ_handler,
};

#define TEST_SIZE	1024
static int test_buf[2][TEST_SIZE];
static u32 counter[2];

static int DMA_input_callback(struct zynq_ipif_dma *dma)
{
	u32 access = dma->access & DMA_BUF_ACCESS_TYPE_MASK;
	u32 buf_max = DATA_BURST * PERIOD_NUM;
	u32 *buf = (u32 *)dma->buf;
	int i;

	for (i = 0; access && i < DATA_BURST && counter[0] < TEST_SIZE; i++)
		buf[dma->io_ptr++ % buf_max] = test_buf[0][counter[0]++];

	if (access == DMA_BUF_ACCESS_TYPE_RWIO) {
		zynq_ipif_dma_write_buffer(dma, (u8 *)&test_buf[0][counter[0]], PERIOD_SIZE);
		counter[0] += DATA_BURST;
	}

	return 0;
}

static int DMA_output_callback(struct zynq_ipif_dma *dma)
{
	u32 access = dma->access & DMA_BUF_ACCESS_TYPE_MASK;
	u32 buf_max = DATA_BURST * PERIOD_NUM;
	u32 *buf = (u32 *)dma->buf;
	int i;

	for (i = 0; access && i < DATA_BURST && counter[1] < TEST_SIZE; i++)
		test_buf[1][counter[1]++] = buf[dma->io_ptr++ % buf_max];

	if (access == DMA_BUF_ACCESS_TYPE_RWIO) {
		zynq_ipif_dma_read_buffer(dma, (u8 *)&test_buf[1][counter[1]], PERIOD_SIZE);
		counter[1] += DATA_BURST;
	}

	return 0;
}

static int DMA_input_condition(struct zynq_ipif_dma *dma)
{
	return counter[0] < TEST_SIZE;
}

static int DMA_output_condition(struct zynq_ipif_dma *dma)
{
	return counter[1] < TEST_SIZE;
}

static struct zynq_ipif_dma_config dma_config[] = {
	{
		.reg_addr = 0x200,
		.buf_size = PERIOD_SIZE,
		.buf_num = PERIOD_NUM,
		.width = DATA_WIDTH,
		.burst = 1,
		.cyclic = 1,
		.access = DMA_BUF_ACCESS_TYPE_RWIO,
		.direction = DMA_DIR_IN,
		.condition = DMA_input_condition,
		.callback = DMA_input_callback,
	},
	{
		/* Skip DMA 1 */
	},
	{
		.reg_addr = 0x208,
		.buf_size = PERIOD_SIZE,
		.buf_num = PERIOD_NUM,
		.width = DATA_WIDTH,
		.burst = 1,
		.cyclic = 1,
		.access = DMA_BUF_ACCESS_TYPE_RWIO,
		.direction = DMA_DIR_OUT,
		.condition = DMA_input_condition,
		.callback = DMA_output_callback,
	},
};

int main()
{
	struct zynq_ipif ipif;
	int i, test = 0x01020304;
	char cmd[256];
	int ret = 0;

	counter[0] = counter[1] = 0;

	for (i = 0; i < ARRAY_SIZE(test_buf[0]); i++)
		test_buf[0][i] = test + i * 0x04040404;

	zynq_ipif_init(&ipif, &ipif_config);

	zynq_ipif_reg_write(&ipif, 0x0, 0x1f);
	zynq_ipif_reg_write(&ipif, 0x0, 0x0);

	zynq_ipif_reg_write(&ipif, 0x1c, 10000);

	zynq_ipif_dma_init(&ipif.dma[0], &dma_config[0]);
	zynq_ipif_dma_init(&ipif.dma[2], &dma_config[2]);

	zynq_ipif_prepare_dma_engine(&ipif);

	/* Fill the ring buffer as the initialization */
	zynq_ipif_dma_write_buffer(&ipif.dma[0], (u8 *)test_buf[0], BUF_SIZE);

	zynq_ipif_dma_enable(&ipif.dma[0], 1);
	zynq_ipif_dma_enable(&ipif.dma[2], 1);

	zynq_ipif_reg_write(&ipif, 0x4, 0x1f);
	zynq_ipif_reg_write(&ipif, 0x8, 0x6667);

	sleep(2);

	zynq_ipif_reg_write(&ipif, 0x8, 0x0);
	zynq_ipif_reg_write(&ipif, 0x4, 0x0);

	zynq_ipif_dma_enable(&ipif.dma[0], 0);
	zynq_ipif_dma_enable(&ipif.dma[2], 0);

	zynq_ipif_unprepare_dma_engine(&ipif);

	zynq_ipif_dma_exit(&ipif.dma[0]);
	zynq_ipif_dma_exit(&ipif.dma[2]);

	for (i = 0; i < ARRAY_SIZE(test_buf[0]); i++)
		if (test_buf[0][i] != test_buf[1][i])
			ret++;

	printf("test %s, %d, %d\n", ret ? "failed" : "succeed",
	       ipif.dma[0].io_ptr, ipif.dma[2].io_ptr);

	zynq_ipif_exit(&ipif);

	printf("dumping results...\n");
	sprintf(cmd, "rm -f dump");
	system(cmd);
	for (i = 0; i < ARRAY_SIZE(test_buf[0]); i++) {
		sprintf(cmd, "echo '%x - %x' >> dump",
				test_buf[0][i], test_buf[1][i]);
		system(cmd);
	}

	return 0;
}
