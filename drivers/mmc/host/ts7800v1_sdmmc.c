// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/mmc/host/ts7800v1_sdmmc.c
 *
 * Copyright (C) 2022 Firas Ashkar <firas.ashkar@savoirfairelinux.com>
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include <linux/bitfield.h>
#include <asm/byteorder.h>

#define DRIVER_NAME "ts7800v1_sdmmc"
#define SDCORE2_SDCMD_REG 0x8
#define SDCORE2_SDDATA_REG 0x4
#define SDCORE2_SDBUS_REG 0xc

#define SD_ACTIVE_SLOT 0x1
#define NUM_SD_SLOTS 0x2
#define NUM_MEM_RESOURCES 0x1
#define MAX_CMD_BYTES 0x6
#define NORM_RESP_BYTES 0x6
#define LONG_RESP_BYTES 0x11
#define BYTE_CLK_CYCLES 0x8
#define DAT03_NIBBLES_PER_CLK_CYCLE 0x1
#define BYTE_CYCLES_MASK 0xff
#define NIBBLE_CLK_CYCLES 0x4
#define CRC7_CYCLES 0x7
#define CRC7_CYCLES_MASK 0x7f
#define CRC16_CYCLES 0x10
#define CRC16_CYCLES_MASK 0xffff
#define CRC_POLY 0x1021

#define CMDENB_DATENB_SDCLKL_SDCMDH_SDDAT03L 0x10
#define CMDENB_DATENB_SDCLKH_SDCMDH_SDDAT03L 0x30
#define CMDENB_DATENB_SDCLKH_SDCMDH_SDDAT03H 0x3f
#define CMDENB_DATENB_SDCLKL_SDCMDH_SDDATL 0x10
#define CMDENB_DATENB_SDCLKL_SDCMDH_SDDAT0L 0x1e
#define CMDENB_DATENB_SDCLKL_SDCMDH_SDDATH 0x1f
#define CMDENB_DATENB_SDCLKH_SDCMDH_SDDAT0L 0x3e
#define CMDTRI_DAT0ENB_SDCLKL_SDCMDH_SDDAT0L 0x5e
#define CMDTRI_DAT0ENB_SDCLKL_SDCMDH_SDDAT0H 0x5f
#define CMDTRI_DAT0ENB_SDCLKH_SDCMDH_SDDAT0L 0x7e
#define CMDTRI_DAT0ENB_SDCLKH_SDCMDH_SDDAT0H 0x7f
#define CMDENB_DATTRI_SDCLKL_SDCMDL_SDDATH 0x8f
#define CMDENB_DATTRI_SDCLKH_SDCMDL_SDDATH 0xaf
#define CMDENB_DATTRI_SDCLKL_SDCMDH_SDDATH 0x9f
#define CMDENB_DATTRI_SDCLKH_SDCMDH_SDDATH 0xbf
#define CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH 0xdf
#define CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH 0xff

#define DATSSP_4BIT (1 << 5)
#define SD_HC BIT(6)
#define SD_MULTI_BLK BIT(7)
#define SD_LOWSPEED BIT(8)
#define SD_SELECTED BIT(9)
#define SD_RESET BIT(10)

#define MAX_RESP_TIMEOUT_MICROSECS 500
#define MAX_BUSY_TIMEOUT_MICROSECS 5000
#define MAX_BLK_SIZE 0x200
#define MAX_BLK_COUNT 0x400
#define MAX_BLK_SIZE_DWORDS 0x80
#define MAX_BLK_SIZE_NIBBLES 0x400

/* TS7800v SD/MMC FIFO size */
#define MAX_SEG_SIZE 0x1000
#define MAX_SEGS 0x400

enum bit_endianness { LE_ENDIAN, BE_ENDIAN };

struct ts7800v1_sdmmc_slot {
	bool sd_detect;
	bool sd_wprot;
	u32 sd_state;
	u32 cmd_timeout;
	u8 *rw_dma_buf;
	u32 blk_buf_cycle_indx;
	u32 blk_buf_nibble_indx;
	int sg_count;
	u8 response[LONG_RESP_BYTES];
	u8 cmdptr[MAX_CMD_BYTES];
};

struct ts7800v1_sdmmc_host {
	struct mmc_host *mmc_host;
	unsigned int sdbusy_irq;
	u8 hw_version;
	void __iomem *base_iomem;
	struct mutex mutex_lock;
	spinlock_t bh_lock;
	struct ts7800v1_sdmmc_slot sd_slot[NUM_SD_SLOTS];
};

static inline void add_1readb_delay(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	readb(ts_sdmmc_host->base_iomem);
}

static inline void add_2readb_delay(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	readb(ts_sdmmc_host->base_iomem);
	readb(ts_sdmmc_host->base_iomem);
}

static inline void
add_2clk_cycles_slow(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	u8 i;

	for (i = 0; i < 2; ++i) {
		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		add_2readb_delay(ts_sdmmc_host);

		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		add_2readb_delay(ts_sdmmc_host);
	}
}

static inline void
add_2clk_cycles_high(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	u8 i;

	for (i = 0; i < 2; ++i) {
		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		add_1readb_delay(ts_sdmmc_host);

		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		add_1readb_delay(ts_sdmmc_host);
	}
}

static inline u16 ts7800v1_sdmmc_ucrc16(u16 crc_in, u8 incr)
{
	u16 xor = crc_in >> 15;
	u16 out = crc_in << 1;

	if (incr)
		out++;

	if (xor)
		out ^= CRC_POLY;

	return out;
}

static u16 ts7800v1_sdmmc_crc16(const u8 *data, u16 size)
{
	u16 crc;
	u8 i;

	for (crc = 0; size > 0; size--, data++)
		for (i = 0x80; i; i >>= 1)
			crc = ts7800v1_sdmmc_ucrc16(crc, *data & i);

	for (i = 0; i < 16; i++)
		crc = ts7800v1_sdmmc_ucrc16(crc, 0);

	return crc;
}

static inline u8 ts7800v1_sdmmc_crc7(u8 crc, const u8 *data, size_t len,
				     enum bit_endianness crc7en)
{
	size_t i, lenbe = len - 1;
	u8 ibit, c;

	if (crc7en == LE_ENDIAN) {
		for (i = 0; i < len; i++) {
			c = data[i];
			for (ibit = 0; ibit < 8; ibit++) {
				crc <<= 1;
				if ((c ^ crc) & 0x80)
					crc ^= 0x09;

				c <<= 1;
			}

			crc &= 0x7F;
		}
	} else {
		for (i = 0; i < len; i++) {
			c = data[lenbe - i];
			for (ibit = 0; ibit < 8; ibit++) {
				crc <<= 1;
				if ((c ^ crc) & 0x80)
					crc ^= 0x09;

				c <<= 1;
			}

			crc &= 0x7F;
		}
	}

	return crc;
}

static inline void lowspeed_mkcommand(u8 cmdindx, u32 arg, u8 *retcmd)
{
	retcmd[0] = BIT(6) | cmdindx;
	retcmd[1] = arg >> 24;
	retcmd[2] = arg >> 16;
	retcmd[3] = arg >> 8;
	retcmd[4] = arg;
	retcmd[5] =
		(0x1 | (ts7800v1_sdmmc_crc7(0, retcmd, 0x5, LE_ENDIAN) << 1));
}

/*
 * return 0 : 8 bit TS-SDCORE v1
 * return 1 : 8 bit 4x8 TS-SDCORE v2
 * return 2 : 32 bit 4x32 TS-SDCORE v2 (TS-7800v1 hw_version 0x2)
 * return 3 : 16 bit 4x32 TS-SDCORE v2
 * return 4 : 8 bit 4x32 TS-SDCORE v2
 */
static int ts7800v1_sdmmc_hw_version(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	u8 a, b;
	u32 c;
	u16 d;
	int ret;

	/*
	 * Bit-30 On TS-SDCORE 2, this bit is stuck 0. On TS-SDCORE 1, this bit is read/write.
	 * This can be used for detecting which hardware core is present.
	 */
	a = readb(ts_sdmmc_host->base_iomem + 0x3);

	writeb((a | BIT(6)), ts_sdmmc_host->base_iomem + 0x3);

	b = readb(ts_sdmmc_host->base_iomem + 0x3);

	/* restore */
	writeb(a, ts_sdmmc_host->base_iomem + 0x3);

	if ((a & BIT(6)) ^ (b & BIT(6))) {
		ret = 0;
		goto print_out;
	} else if (a & BIT(6)) {
		ret = 1;
		goto print_out;
	}

	c = readl(ts_sdmmc_host->base_iomem + SDCORE2_SDBUS_REG);
	d = readw(ts_sdmmc_host->base_iomem + SDCORE2_SDBUS_REG);

	if ((c & BIT(6)) && (d & BIT(6))) {
		ret = 2;
		goto print_out;
	}

	a = readb(ts_sdmmc_host->base_iomem + SDCORE2_SDBUS_REG);
	if (a & BIT(6)) {
		ret = 3;
		goto print_out;
	} else {
		ret = 4;
		goto print_out;
	}

print_out:
	dev_info(mmc_dev(ts_sdmmc_host->mmc_host), "Detected SDCoreV%d\n", ret);
	return ret;
}

static inline u8 get_clksel(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	return readb(ts_sdmmc_host->base_iomem + 0x2) & GENMASK(2, 0);
}

static inline void set_clksel(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			      u8 slot)
{
	u8 a;
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	a = get_clksel(ts_sdmmc_host);
	a &= ~(GENMASK(2, 0));
	a |= (slot & GENMASK(2, 0));
	writeb(a, ts_sdmmc_host->base_iomem + 0x2);
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);
}

static u32 set_clkspd(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
		      struct ts7800v1_sdmmc_slot *pslot, bool high_speed)
{
	u8 a;

	/* since this is a single host multi slot/card state machine */
	/* always change clock frequency for current slot */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	pslot->sd_state &= ~(SD_LOWSPEED);
	a = readb(ts_sdmmc_host->base_iomem + 0x1);
	a &= ~(BIT(5));
	if (high_speed) {
		a |= BIT(5);
		writeb(a, ts_sdmmc_host->base_iomem + 0x1);
	} else {
		writeb(a, ts_sdmmc_host->base_iomem + 0x1);
		pslot->sd_state |= SD_LOWSPEED;
	}
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	return (pslot->sd_state & SD_LOWSPEED);
}

static u32 set_mlt_rdwr(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			struct ts7800v1_sdmmc_slot *pslot, bool multi_word)
{
	u8 a;

	/* since this is a single host multi slot/card state machine */
	/* always change read/write type for current slot */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	pslot->sd_state &= ~SD_MULTI_BLK;
	a = readb(ts_sdmmc_host->base_iomem + 0x1);
	a &= ~(GENMASK(4, 3));
	if (multi_word) {
		a |= GENMASK(4, 3);
		writeb(a, ts_sdmmc_host->base_iomem + 0x1);
		pslot->sd_state |= SD_MULTI_BLK;
	} else {
		writeb(a, ts_sdmmc_host->base_iomem + 0x1);
	}
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	return (a & GENMASK(4, 3));
}

static int activate_slot_clk(struct ts7800v1_sdmmc_host *ts_sdmmc_host, u8 slot)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	bool high_speed, multi_rw;

	/* Are we already selected? */
	if ((pslot->sd_state & (SD_SELECTED | SD_RESET)) == SD_SELECTED)
		return 0;

	/* Change clock routing */
	set_clksel(ts_sdmmc_host, slot);

	/* Change clock freq/multi-blk read/write */
	multi_rw = (pslot->sd_state & SD_MULTI_BLK) ? true : false;
	set_mlt_rdwr(ts_sdmmc_host, pslot, multi_rw);
	high_speed = (pslot->sd_state & SD_LOWSPEED) ? false : true;
	set_clkspd(ts_sdmmc_host, pslot, high_speed);

	/* mark us as selected */
	pslot->sd_state |= SD_SELECTED;

	return 0;
}

static int card_reset(struct ts7800v1_sdmmc_host *ts_sdmmc_host, u8 slot)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	u16 i;
	u8 a;

	/* reset sdmmc state bits */
	pslot->sd_state = 0x0;

	/* start with low speed */
	pslot->sd_state |= SD_LOWSPEED;

	/* select which LUN gets the clocks */
	activate_slot_clk(ts_sdmmc_host, slot);

	/* disable clk, cmd and dat[0-3] => power off SD card */
	writeb(0x0, ts_sdmmc_host->base_iomem);
	msleep(100);

	writeb(CMDENB_DATTRI_SDCLKH_SDCMDH_SDDATH, ts_sdmmc_host->base_iomem);
	usleep_range(200, 300);
	writeb(CMDENB_DATTRI_SDCLKL_SDCMDL_SDDATH, ts_sdmmc_host->base_iomem);
	msleep(100);

	// generate free 750-clocks cycles for the cards
	for (i = 0; i < 750; ++i) {
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* 2-delay reads */
		add_2readb_delay(ts_sdmmc_host);

		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* 2-delay reads */
		add_2readb_delay(ts_sdmmc_host);
	}

	/* reset any timeout/crc conditions */
	a = readb(ts_sdmmc_host->base_iomem + 0x1);

	/* set card-detect and write-protect */
	pslot->sd_detect = (a & BIT(0)) ? true : false;
	pslot->sd_wprot = (a & BIT(1)) ? true : false;

	pslot->sd_state &= ~(SD_RESET);

	return 0;
}

/* set/clear bit location in any contiguous memory buffer/fifo
 * this function assumes pfifo content are cleared prior to calling it
 */
static inline void set_fifo_bit(u8 *pfifo, uint32_t cycle, u8 value)
{
	u32 byte_indx = cycle >> 3;
	u8 bit_indx = cycle - (byte_indx << 3);
	if (value)
		pfifo[byte_indx] |= BIT(bit_indx);
	else
		pfifo[byte_indx] &= ~BIT(bit_indx);
}

/* reversed big endian set/clear bit location in any contiguous memory buffer/fifo
 * this function assumes pfifo content are cleared prior to calling it
 */
static inline void set_fifo_bit_reversed(u8 *pfifo, uint32_t cycle, u8 value)
{
	u32 byte_indx = cycle >> 3;
	u8 bit_indx = 7 - (cycle - (byte_indx << 3));
	if (value)
		pfifo[byte_indx] |= BIT(bit_indx);
	else
		pfifo[byte_indx] &= ~BIT(bit_indx);
}

static inline void set_fifo_nibble_reversed(u8 *pfifo, uint32_t nibble_cycle,
					    u8 value)
{
	u32 byte_indx = nibble_cycle >> 1;
	u8 nibble_indx = (nibble_cycle - (byte_indx << 1));
	if (nibble_indx)
		pfifo[byte_indx] |= (value & GENMASK(3, 0));
	else
		pfifo[byte_indx] |= ((value & GENMASK(3, 0)) << 0x4);
}

/* bitbang read SD_CMD/SD_DAT (high speed) */
static inline void
read_sd_cmd_sd_dat_highspeed(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			     u8 *sdcmd_buffer, u8 *sddat_buffer,
			     u32 sdcmd_cycles, u32 sddat_nibble_cycles, u8 slot)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	bool dat_started = false;
	u32 i, sdcmd_msb_indx = sdcmd_cycles - 1;
	u8 x;

	/* set cmd start bit */
	if (sdcmd_buffer != NULL) {
		set_fifo_bit(sdcmd_buffer, sdcmd_msb_indx, 0x0);
	}

	/* read/sample sdcmd/sddat0 bits */
	for (i = 1; i < sdcmd_cycles; i++) {
		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		add_1readb_delay(ts_sdmmc_host);

		/* read/sample */
		x = readb(ts_sdmmc_host->base_iomem);

		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* set cmd bit */
		if (sdcmd_buffer != NULL) {
			set_fifo_bit(sdcmd_buffer, sdcmd_msb_indx - i,
				     ((x & BIT(0x4)) >> 0x4));
		}

		/* set dat0-dat3 bits */
		if (sddat_buffer != NULL) {
			if (dat_started) {
				if (pslot->blk_buf_nibble_indx <
				    sddat_nibble_cycles) {
					set_fifo_nibble_reversed(
						sddat_buffer,
						pslot->blk_buf_nibble_indx,
						(x & GENMASK(3, 0)));
					pslot->blk_buf_nibble_indx++;
				}

			} else {
				/* ignore start bit */
				if ((x & GENMASK(3, 0)) == 0x0)
					dat_started = true;
			}
		}
	}

	/* continue reading remaining dat0-dat3 until next block boundary */
	if (sddat_buffer != NULL && dat_started) {
		while (pslot->blk_buf_nibble_indx < sddat_nibble_cycles &&
		       pslot->blk_buf_nibble_indx < MAX_BLK_SIZE_NIBBLES) {
			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			add_1readb_delay(ts_sdmmc_host);

			/* read/sample */
			x = readb(ts_sdmmc_host->base_iomem);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			set_fifo_nibble_reversed(sddat_buffer,
						 pslot->blk_buf_nibble_indx,
						 (x & GENMASK(3, 0)));
			pslot->blk_buf_nibble_indx++;
		}
	}

	if (pslot->blk_buf_nibble_indx == MAX_BLK_SIZE_NIBBLES) {
		/* read/consume sd_dat CRC16 */
		for (i = 0; i < 0x20; i++) {
			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			add_1readb_delay(ts_sdmmc_host);

			/* read/sample */
			x = readb(ts_sdmmc_host->base_iomem);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			add_1readb_delay(ts_sdmmc_host);
		}
	}
}

/* bitbang read SD_CMD/SD_DAT (low speed) */
static inline void
read_sdcmd_sddat0_lowspeed(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			   u8 *sdcmd_buffer, u8 *sddat0_buffer,
			   u32 sdcmd_cycles, u32 sddat0_cycles, u8 slot)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	bool dat0_started = false;
	u32 i, sdcmd_msb_indx = sdcmd_cycles - 1;
	u8 x;

	/* set cmd start bit */
	if (sdcmd_buffer != NULL) {
		set_fifo_bit(sdcmd_buffer, sdcmd_msb_indx, 0x0);
	}

	/* read/sample sdcmd/sddat0 bits */
	for (i = 1; i < sdcmd_cycles; i++) {
		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* 2-delay read */
		add_2readb_delay(ts_sdmmc_host);

		/* read/sample */
		x = readb(ts_sdmmc_host->base_iomem);

		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* set cmd bit */
		if (sdcmd_buffer != NULL) {
			set_fifo_bit(sdcmd_buffer, sdcmd_msb_indx - i,
				     ((x & BIT(0x4)) >> 0x4));
		}

		/* set dat0 bit */
		if (sddat0_buffer != NULL) {
			if (dat0_started) {
				if (pslot->blk_buf_cycle_indx < sddat0_cycles) {
					set_fifo_bit_reversed(
						sddat0_buffer,
						pslot->blk_buf_cycle_indx,
						(x & BIT(0x0)));
					pslot->blk_buf_cycle_indx++;
				}

			} else {
				/* ignore start bit */
				if ((x & GENMASK(3, 0)) == 0xe)
					dat0_started = true;
			}
		}

		/* 1-delay read */
		add_1readb_delay(ts_sdmmc_host);
	}
}

/* read/continue previously started bit read operation */
static inline void
read_continue_sddat0_lowspeed(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			      u8 *sddat0_buffer, u32 start_cycle,
			      u32 sddat0_cycles, u8 slot, bool reverse)
{
	u32 i, sddat0_msb_indx = sddat0_cycles - 1;

	/* reverse bit/byte order rw DMA buffer */
	if (reverse) {
		/* read/sample sdcmd/sddat0 bits */
		for (i = start_cycle; i < sddat0_cycles; i++) {
			u8 x;
			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay read */
			add_1readb_delay(ts_sdmmc_host);

			/* read/sample */
			x = readb(ts_sdmmc_host->base_iomem);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* set dat0 bit */
			if (sddat0_buffer != NULL) {
				set_fifo_bit_reversed(sddat0_buffer, i,
						      (x & BIT(0x0)));
			}

			/* 1-delay read */
			add_1readb_delay(ts_sdmmc_host);
		}
	} else {
		/* read/sample sdcmd/sddat0 bits */
		for (i = start_cycle; i < sddat0_cycles; i++) {
			u8 x;
			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay read */
			add_2readb_delay(ts_sdmmc_host);

			/* read/sample */
			x = readb(ts_sdmmc_host->base_iomem);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* set dat0 bit */
			if (sddat0_buffer != NULL) {
				set_fifo_bit(sddat0_buffer, sddat0_msb_indx - i,
					     (x & BIT(0x0)));
			}

			/* 1-delay read */
			add_1readb_delay(ts_sdmmc_host);
		}
	}
}

/* write/serialize bit to dat0 */
static inline void
write_sample_sddat0_lowspeed(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			     u32 cycles, u8 x8)
{
	u32 j;
	u8 bit_mask;

	BUG_ON(cycles > 0x20);

	bit_mask = BIT(cycles - 1);

	for (j = 0; j < cycles; j++) {
		if ((x8 << j) & bit_mask) {
			/* toggle high slow clk-line */
			writeb(CMDTRI_DAT0ENB_SDCLKH_SDCMDH_SDDAT0H,
			       ts_sdmmc_host->base_iomem);

			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DAT0ENB_SDCLKL_SDCMDH_SDDAT0H,
			       ts_sdmmc_host->base_iomem);

			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

		} else {
			/* toggle high slow clk-line */
			writeb(CMDTRI_DAT0ENB_SDCLKH_SDCMDH_SDDAT0L,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DAT0ENB_SDCLKL_SDCMDH_SDDAT0L,
			       ts_sdmmc_host->base_iomem);

			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);
		}
	}
}

static inline void sd_cmd_write(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
				u8 offset, u32 opcode, u32 arg)
{
	writeb(CMDENB_DATTRI_SDCLKH_SDCMDH_SDDATH, ts_sdmmc_host->base_iomem);

	if (offset == 0x20) {
		u32 x = 0x0;
		x = ((BIT(6) | opcode) & 0xff) << 24;
		x |= ((arg >> 24) & 0xff) << 16;
		x |= ((arg >> 16) & 0xff) << 8;
		x |= ((arg >> 8) & 0xff);

		writel(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);

		/* send remaining 1-byte of arg */
		/* NOTE: CRC7 + STOP bit are added automatically */
		x = (arg & 0xff) << 24;
		writel(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);

	} else if (offset == 0x10) {
		u16 x = 0x0;
		x = (opcode & 0xff) << 8;
		x |= ((arg >> 24) & 0xff);
		writew(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);

		x = ((arg >> 16) & 0xff) << 8;
		x |= ((arg >> 8) & 0xff);
		writew(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);

		x = (arg & 0xff) << 8;
		writew(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
	} else {
		u8 x = 0x0;
		x = (opcode & 0xff);
		writeb(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
		x = ((arg >> 24) & 0xff);
		writeb(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
		x = ((arg >> 16) & 0xff);
		writeb(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
		x = ((arg >> 8) & 0xff);
		writeb(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
		x = ((arg)&0xff);
		writeb(x, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
	}

	writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH, ts_sdmmc_host->base_iomem);
}

static inline void sd_cmd_read(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
			       struct ts7800v1_sdmmc_slot *pslot,
			       u8 resp_len_bytes, size_t sz)
{
	u8 i, j;
	u8 x8, resp_last_byte = resp_len_bytes - 1;
	u16 x16;
	u32 x32;

	for (i = 0; i < resp_len_bytes; i++) {
		if (!(i % sz)) {
			u8 shift = (sz - 1) << 3;
			if (sz == sizeof(x8)) {
				x8 = readb(ts_sdmmc_host->base_iomem +
					   SDCORE2_SDCMD_REG);
				for (j = i; j < i + sz; ++j) {
					if (j < resp_len_bytes) {
						pslot->response[resp_last_byte -
								j] =
							(x8 >> shift) & 0xff;
						shift -= 8;
					}
				}
			} else if (sz == sizeof(x16)) {
				x16 = readw(ts_sdmmc_host->base_iomem +
					    SDCORE2_SDCMD_REG);
				for (j = i; j < i + sz; ++j) {
					if (j < resp_len_bytes) {
						pslot->response[resp_last_byte -
								j] =
							(x16 >> shift) & 0xff;
						shift -= 8;
					}
				}
			} else {
				x32 = (readl(ts_sdmmc_host->base_iomem +
					     SDCORE2_SDCMD_REG));

				for (j = i; j < i + sz; ++j) {
					if (j < resp_len_bytes) {
						pslot->response[resp_last_byte -
								j] =
							(x32 >> shift) & 0xff;

						shift -= 8;
					}
				}
			}
		}
	}
}

/* This function should be called after holding spin lock */
static inline void send_serialize_cmd(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
				      struct ts7800v1_sdmmc_slot *pslot,
				      u32 opcode, u32 arg, bool low_speed)
{
	u8 i, j;

	if (low_speed) {
		// Build command packet
		lowspeed_mkcommand((opcode & 0xff), arg, pslot->cmdptr);

		add_2clk_cycles_slow(ts_sdmmc_host);

		/* Send command on slow cmd-line */
		for (i = 0; i < MAX_CMD_BYTES; i++) {
			u8 b = pslot->cmdptr[i];
			u8 x;

			for (j = 0; j < 8; j++) {
				/* set cmd bits at low clk */
				x = CMDENB_DATTRI_SDCLKL_SDCMDL_SDDATH |
				    ((b & BIT(7)) >> 0x3);
				b = b << 1;
				/* write one bit of cmdptr to slow cmd-line */
				writeb(x, ts_sdmmc_host->base_iomem);
				/* 2-delay reads */
				add_2readb_delay(ts_sdmmc_host);

				/* toggle low slow clk-line */
				x |= BIT(5);
				writeb(x, ts_sdmmc_host->base_iomem);
				/* 2-delay reads */
				add_2readb_delay(ts_sdmmc_host);
			}
		}

		/* toggle clk low */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

	} else {
		add_2clk_cycles_high(ts_sdmmc_host);

		sd_cmd_write(ts_sdmmc_host, 0x20, opcode, arg);
	}
}

/* This function should be called after holding mutex lock */
static inline void wait_for_response(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
				     struct ts7800v1_sdmmc_slot *pslot,
				     bool low_speed)
{
	pslot->cmd_timeout = 0x0;
	for (pslot->cmd_timeout = 0;
	     pslot->cmd_timeout < MAX_RESP_TIMEOUT_MICROSECS;
	     ++pslot->cmd_timeout) {
		u8 x;

		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* add 1-delay */
		if (low_speed)
			add_1readb_delay(ts_sdmmc_host);

		/* read/sample sd_cmd state */
		x = readb(ts_sdmmc_host->base_iomem);

		/* toggle low slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* add 2-delay */
		if (low_speed)
			add_2readb_delay(ts_sdmmc_host);

		if ((x & 0x10) == 0x0)
			break;

		usleep_range(1, 2);
	}
}

static int send_cmd_recv_resp_simple(struct ts7800v1_sdmmc_host *ts_sdmmc_host,
				     u8 slot, u32 cmd_opcode, u32 cmd_arg,
				     unsigned int cmd_flags, int *cmd_error,
				     u32 *cmd_resp)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	u8 stat, resp_len_bytes, sent_resp_crc7, calc_resp_crc7;
	bool low_speed;
	int ret = 0x0, i;

	/* initial ok state, following are not pre-set by default */
	pslot->sg_count = -1;
	pslot->cmd_timeout = 0x0;
	pslot->blk_buf_cycle_indx = pslot->blk_buf_nibble_indx = 0x0;

	/* low speed = sample on cmd-line, dat0-line */
	low_speed = (pslot->sd_state & SD_LOWSPEED) ? true : false;

	activate_slot_clk(ts_sdmmc_host, slot);

	/* serialize command */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	send_serialize_cmd(ts_sdmmc_host, pslot, cmd_opcode, cmd_arg,
			   low_speed);
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	switch ((cmd_flags & (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			      MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
	case MMC_RSP_NONE:
		resp_len_bytes = 0x0;
		goto done;
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		resp_len_bytes = NORM_RESP_BYTES;
		break;
	case MMC_RSP_R2:
		resp_len_bytes = LONG_RESP_BYTES;
		break;
	default:
		dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
			 "Warning, Invalid response type\n");
		*cmd_error = ret = -EINVAL;
		goto done;
	}

	if (cmd_flags & MMC_RSP_PRESENT) {
		/* wait for response, i.e. start bit=0 on slow sd_cmd */
		mutex_lock(&ts_sdmmc_host->mutex_lock);
		wait_for_response(ts_sdmmc_host, pslot, low_speed);
		mutex_unlock(&ts_sdmmc_host->mutex_lock);

		if (pslot->cmd_timeout >= MAX_RESP_TIMEOUT_MICROSECS) {
			*cmd_error = ret = -ETIMEDOUT;
			goto done;
		}

		/* serialize/consume cmd response */
		spin_lock_bh(&ts_sdmmc_host->bh_lock);
		if (!low_speed && (pslot->sd_state & DATSSP_4BIT)) {
			sd_cmd_read(ts_sdmmc_host, pslot, resp_len_bytes,
				    sizeof(u32));

		} else {
			/* NOTE: response includes crc7 and stop bit */
			read_sdcmd_sddat0_lowspeed(
				ts_sdmmc_host, pslot->response, NULL,
				(resp_len_bytes * BYTE_CLK_CYCLES), 0x0, slot);
		}

		spin_unlock_bh(&ts_sdmmc_host->bh_lock);

		/* process data outside of spin locks */
		if (cmd_flags & MMC_RSP_136) {
			/* R2, copy 12-bytes/3-double-words of argument including internal CRC7 */
			memcpy(&cmd_resp[0], &pslot->response[12], 0x4);
			memcpy(&cmd_resp[1], &pslot->response[8], 0x4);
			memcpy(&cmd_resp[2], &pslot->response[4], 0x4);
			memcpy(&cmd_resp[3], &pslot->response[0], 0x4);
		} else {
			/* R1, R1b, R3, R4, R5, R6 4-bytes arguments only*/
			memcpy(cmd_resp, &pslot->response[1], 0x4);
		}

		/* resp crc7 check */
		switch ((cmd_flags &
			 (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			  MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
		case MMC_RSP_R1:
		case MMC_RSP_R1B:
			sent_resp_crc7 = pslot->response[0] >> 1;
			calc_resp_crc7 =
				ts7800v1_sdmmc_crc7(0, &pslot->response[1],
						    resp_len_bytes - 1,
						    BE_ENDIAN);

			if (sent_resp_crc7 != calc_resp_crc7) {
				*cmd_error = ret = -EILSEQ;
				goto done;
			}
			break;
		default:
			/* no crc7 check*/
			break;
		}
	}

	/* serialize/consume card's busy response if any */
	if (cmd_flags & MMC_RSP_BUSY) {
		stat = 0x0;
		/* reset cmd time-out */
		pslot->cmd_timeout = 0x0;

		mutex_lock(&ts_sdmmc_host->mutex_lock);

		while ((stat & 0x7) != 0x7) {
			if (pslot->cmd_timeout++ >=
			    MAX_BUSY_TIMEOUT_MICROSECS) {
				*cmd_error = ret = -ETIMEDOUT;
				goto done;
			}

			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			stat = stat << 1;
			stat |= readb(ts_sdmmc_host->base_iomem) & 0x1;

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			usleep_range(15, 25);
		}

		mutex_unlock(&ts_sdmmc_host->mutex_lock);
	}

	*cmd_error = ret = 0x0;

done:

	// 8 clocks before stopping
	spin_lock_bh(&ts_sdmmc_host->bh_lock);

	if (low_speed)
		for (i = 0; i < 8; i++) {
			/* toggle hi slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);
		}
	else {
		/* send 8-bits on fast clk-line */
		writeb(0xff, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
	}

	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	return ret;
}

static int send_cmd_recv_resp_read_blk(
	struct ts7800v1_sdmmc_host *ts_sdmmc_host, u8 slot, u32 cmd_opcode,
	u32 cmd_arg, unsigned int cmd_flags, int *cmd_error, u32 *cmd_resp,
	unsigned int data_blksz, u32 data_offset, int *data_error)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	u32 blksz_dwords = data_blksz >> 2;
	u16 sent_dat0_crc16, calc_crc16;
	u8 stat, resp_len_bytes, sent_resp_crc7, calc_resp_crc7,
		*dat0_sent_crc16_buf = NULL;
	bool low_speed, data_read_done;
	int ret = 0x0, i;

	/* initial ok state, following are not pre-set by default */
	pslot->sg_count = -1;
	pslot->cmd_timeout = 0x0;
	pslot->blk_buf_cycle_indx = pslot->blk_buf_nibble_indx = 0x0;
	data_read_done = false;
	/* clear crc_err/timeout */
	readb(ts_sdmmc_host->base_iomem + 0x1);

	/* low speed = sample on cmd-line, dat0-line */
	low_speed = (pslot->sd_state & SD_LOWSPEED) ? true : false;

	activate_slot_clk(ts_sdmmc_host, slot);

	if (IS_ERR_OR_NULL(pslot->rw_dma_buf)) {
		dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
			 "Error, No allocated DMA read buffer %ld\n",
			 PTR_ERR(pslot->rw_dma_buf));
		*data_error = ret = -ENOMEM;
		goto done;
	}

	dat0_sent_crc16_buf = kzalloc(sizeof(u16), GFP_KERNEL);
	if (IS_ERR_OR_NULL(dat0_sent_crc16_buf)) {
		dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
			 "Error, kzalloc failed with %ld\n",
			 PTR_ERR(dat0_sent_crc16_buf));
		*data_error = ret = -ENOMEM;
		goto done;
	}

	if (!(cmd_flags & MMC_RSP_PRESENT)) {
		dev_warn(
			mmc_dev(ts_sdmmc_host->mmc_host),
			"Error, read block command flgas must have a response\n");
		*cmd_error = ret = -EINVAL;
	} else {
		switch ((cmd_flags &
			 (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			  MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
		case MMC_RSP_NONE:
			resp_len_bytes = 0x0;
			goto done;
		case MMC_RSP_R1:
		case MMC_RSP_R1B:
		case MMC_RSP_R3:
			resp_len_bytes = NORM_RESP_BYTES;
			break;
		case MMC_RSP_R2:
			resp_len_bytes = LONG_RESP_BYTES;
			break;
		default:
			dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
				 "Warning, Invalid response type\n");
			*cmd_error = ret = -EINVAL;
			goto done;
		}
	}

	/* serialize command */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	send_serialize_cmd(ts_sdmmc_host, pslot, cmd_opcode, cmd_arg,
			   low_speed);
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	/* wait for response, i.e. start bit=0 on slow sd_cmd */
	mutex_lock(&ts_sdmmc_host->mutex_lock);
	wait_for_response(ts_sdmmc_host, pslot, low_speed);
	mutex_unlock(&ts_sdmmc_host->mutex_lock);

	if (pslot->cmd_timeout >= MAX_RESP_TIMEOUT_MICROSECS) {
		*cmd_error = ret = -ETIMEDOUT;
		goto done;
	}

	/* serialize/consume cmd response */
	if (!low_speed && (pslot->sd_state & DATSSP_4BIT)) {
		spin_lock_bh(&ts_sdmmc_host->bh_lock);
		read_sd_cmd_sd_dat_highspeed(
			ts_sdmmc_host, pslot->response, pslot->rw_dma_buf,
			resp_len_bytes * BYTE_CLK_CYCLES,
			data_blksz << DAT03_NIBBLES_PER_CLK_CYCLE, slot);

		if (pslot->blk_buf_nibble_indx != 0) {
			if (pslot->blk_buf_nibble_indx < (data_blksz << 1)) {
				u32 blk_buf_nibble_indx_dwords;
				stat = 0x0;

				/* ignore One-cycle Pull-up */
				writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);
				/* read/sample */
				readb(ts_sdmmc_host->base_iomem);
				/* toggle low slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);
				add_1readb_delay(ts_sdmmc_host);

				blk_buf_nibble_indx_dwords =
					pslot->blk_buf_nibble_indx >> 3;
				for (i = blk_buf_nibble_indx_dwords;
				     i < blksz_dwords; ++i) {
					u32 x32;

					if (!(i % MAX_BLK_SIZE_DWORDS)) {
						/* check/clear crc_err/timeout */
						stat = readb(
							ts_sdmmc_host
								->base_iomem +
							0x1);

						if (stat & BIT(2)) {
							spin_unlock_bh(
								&ts_sdmmc_host
									 ->bh_lock);

							dev_warn(
								mmc_dev(ts_sdmmc_host
										->mmc_host),
								"Warning, dat0-3 crc16 mismatch\n");

							*data_error = ret =
								-EILSEQ;

							goto done;
						}

						if (stat & BIT(6)) {
							spin_unlock_bh(
								&ts_sdmmc_host
									 ->bh_lock);

							dev_warn(
								mmc_dev(ts_sdmmc_host
										->mmc_host),
								"Warning, dat0-3 time-out\n");

							*data_error = ret =
								-ETIMEDOUT;

							goto done;
						}
					}

					x32 = readl(ts_sdmmc_host->base_iomem +
						    SDCORE2_SDDATA_REG);

					memcpy(&pslot->rw_dma_buf[i << 2], &x32,
					       sizeof(u32));
				}

				data_read_done = true;
			} else {
				/* already completed reading data blk */
				data_read_done = true;
			}
		}

		spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	} else {
		spin_lock_bh(&ts_sdmmc_host->bh_lock);
		/* NOTE: response includes crc7 and stop bit */
		read_sdcmd_sddat0_lowspeed(ts_sdmmc_host, pslot->response,
					   pslot->rw_dma_buf,
					   (resp_len_bytes * BYTE_CLK_CYCLES),
					   (data_blksz * BYTE_CLK_CYCLES),
					   slot);

		if (pslot->blk_buf_cycle_indx != 0) {
			read_continue_sddat0_lowspeed(
				ts_sdmmc_host, pslot->rw_dma_buf,
				pslot->blk_buf_cycle_indx,
				(data_blksz * BYTE_CLK_CYCLES), slot, true);

			/* read dat0 CRC16 */
			read_continue_sddat0_lowspeed(ts_sdmmc_host,
						      dat0_sent_crc16_buf, 0x0,
						      CRC16_CYCLES, slot,
						      false);

			sent_dat0_crc16 = (dat0_sent_crc16_buf[1] << 0x8) |
					  dat0_sent_crc16_buf[0];

			/* serialize/consume stop bit */
			read_continue_sddat0_lowspeed(ts_sdmmc_host, NULL, 0x0,
						      0x1, slot, false);

			/* check data crc16 */
			calc_crc16 = ts7800v1_sdmmc_crc16(pslot->rw_dma_buf,
							  data_blksz);

			if (calc_crc16 != sent_dat0_crc16) {
				spin_unlock_bh(&ts_sdmmc_host->bh_lock);

				dev_warn(
					mmc_dev(ts_sdmmc_host->mmc_host),
					"Warning, dat0 crc16 mismatch sent-crc16 %#x, calc-crc16 %#x\n",
					sent_dat0_crc16, calc_crc16);

				*data_error = ret = -EILSEQ;
				goto done;
			}

			data_read_done = true;
		}

		spin_unlock_bh(&ts_sdmmc_host->bh_lock);
	}

	/* serialize/consume card's busy response if any */
	if (cmd_flags & MMC_RSP_BUSY) {
		stat = 0x0;
		/* reset cmd time-out */
		pslot->cmd_timeout = 0x0;

		mutex_lock(&ts_sdmmc_host->mutex_lock);

		while ((stat & 0x7) != 0x7) {
			if (pslot->cmd_timeout++ >=
			    MAX_BUSY_TIMEOUT_MICROSECS) {
				*cmd_error = ret = -ETIMEDOUT;
				goto done;
			}

			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			stat = stat << 1;
			stat |= readb(ts_sdmmc_host->base_iomem) & 0x1;

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			usleep_range(15, 25);
		}

		mutex_unlock(&ts_sdmmc_host->mutex_lock);
	}

	/* process response outside of spin locks */
	if (cmd_flags & MMC_RSP_136) {
		/* R2, copy 12-bytes/3-double-words of argument including internal CRC7 */
		memcpy(&cmd_resp[0], &pslot->response[12], 0x4);
		memcpy(&cmd_resp[1], &pslot->response[8], 0x4);
		memcpy(&cmd_resp[2], &pslot->response[4], 0x4);
		memcpy(&cmd_resp[3], &pslot->response[0], 0x4);
	} else {
		/* R1, R1b, R3, R4, R5, R6 4-bytes arguments only*/
		memcpy(cmd_resp, &pslot->response[1], 0x4);
	}

	/* resp crc7 check */
	switch ((cmd_flags & (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			      MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
		sent_resp_crc7 = pslot->response[0] >> 1;
		calc_resp_crc7 = ts7800v1_sdmmc_crc7(
			0, &pslot->response[1], resp_len_bytes - 1, BE_ENDIAN);

		if (sent_resp_crc7 != calc_resp_crc7) {
			*cmd_error = ret = -EILSEQ;
			goto done;
		}
		break;
	default:
		/* no crc7 check*/
		break;
	}

	if (!data_read_done) {
		/* wait for data lines to become low, i.e. dat[0-3]=0x0 */
		if (!low_speed && (pslot->sd_state & DATSSP_4BIT)) {
			mutex_lock(&ts_sdmmc_host->mutex_lock);
			/* wait for start block token */
			pslot->cmd_timeout = 0x0;
			stat = 0xff;

			while ((stat & 0xf) == 0xf) {
				if (pslot->cmd_timeout++ >=
				    MAX_BUSY_TIMEOUT_MICROSECS) {
					/* reset time-out before going to done, since cmd may have been successful and only data-transfer failed */
					pslot->cmd_timeout = 0x0;
					*data_error = ret = -ETIMEDOUT;

					mutex_unlock(
						&ts_sdmmc_host->mutex_lock);
					goto done;
				}

				/* toggle high slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				/* 1-delay reads */
				add_1readb_delay(ts_sdmmc_host);

				/* toggle low slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				stat = readb(ts_sdmmc_host->base_iomem);

				usleep_range(15, 25);
			}

			mutex_unlock(&ts_sdmmc_host->mutex_lock);

			spin_lock_bh(&ts_sdmmc_host->bh_lock);

			/* ignore One-cycle Pull-up */
			for (i = 0; i < 0x1; ++i) {
				/* toggle high slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				add_1readb_delay(ts_sdmmc_host);

				/* read/sample */
				stat = (readb(ts_sdmmc_host->base_iomem) & 0xf);

				/* toggle low slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				/* add 2-clk delay */
				add_2readb_delay(ts_sdmmc_host);
			}

			for (i = 0; i < blksz_dwords; i++) {
				u32 x32;

				if (!(i % MAX_BLK_SIZE_DWORDS)) {
					/* check/clear crc_err/timeout */
					stat = readb(ts_sdmmc_host->base_iomem +
						     0x1);

					if (stat & BIT(2)) {
						spin_unlock_bh(
							&ts_sdmmc_host->bh_lock);

						dev_warn(
							mmc_dev(ts_sdmmc_host
									->mmc_host),
							"Warning, dat0-3 crc16 mismatch\n");

						*data_error = ret = -EILSEQ;

						goto done;
					}

					if (stat & BIT(6)) {
						spin_unlock_bh(
							&ts_sdmmc_host->bh_lock);

						dev_warn(
							mmc_dev(ts_sdmmc_host
									->mmc_host),
							"Warning, dat0-3 time-out\n");

						*data_error = ret = -ETIMEDOUT;

						goto done;
					}
				}

				x32 = readl(ts_sdmmc_host->base_iomem +
					    SDCORE2_SDDATA_REG);

				memcpy(&pslot->rw_dma_buf[i << 2], &x32,
				       sizeof(u32));
			}

			spin_unlock_bh(&ts_sdmmc_host->bh_lock);

		} else {
			pslot->cmd_timeout = 0x0;
			stat = 0xff;

			mutex_lock(&ts_sdmmc_host->mutex_lock);
			while ((stat & 0xf) == 0xf) {
				if (pslot->cmd_timeout++ >=
				    MAX_BUSY_TIMEOUT_MICROSECS) {
					/* reset time-out before going to done, since cmd may have been successful and only data-transfer failed */
					pslot->cmd_timeout = 0x0;
					*data_error = ret = -ETIMEDOUT;

					mutex_unlock(
						&ts_sdmmc_host->mutex_lock);
					goto done;
				}

				/* toggle high slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				/* 2-delay reads */
				add_2readb_delay(ts_sdmmc_host);

				/* toggle low slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				add_1readb_delay(ts_sdmmc_host);

				stat = readb(ts_sdmmc_host->base_iomem);

				usleep_range(15, 25);
			}

			mutex_unlock(&ts_sdmmc_host->mutex_lock);

			spin_lock_bh(&ts_sdmmc_host->bh_lock);
			/* ignore start bit */
			for (i = 0; i < 0x1; ++i) {
				/* toggle high slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				add_1readb_delay(ts_sdmmc_host);

				/* read/sample */
				stat = (readb(ts_sdmmc_host->base_iomem) & 0xf);

				/* toggle low slow clk-line */
				writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
				       ts_sdmmc_host->base_iomem);

				/* add 2-clk delay */
				add_2readb_delay(ts_sdmmc_host);
			}

			/* consume data, read dat0 */
			read_continue_sddat0_lowspeed(
				ts_sdmmc_host, pslot->rw_dma_buf, 0x0,
				data_blksz * BYTE_CLK_CYCLES, slot, true);

			/* read dat0 CRC16 */
			read_continue_sddat0_lowspeed(ts_sdmmc_host,
						      dat0_sent_crc16_buf, 0x0,
						      CRC16_CYCLES, slot,
						      false);

			sent_dat0_crc16 = (dat0_sent_crc16_buf[1] << 0x8) |
					  dat0_sent_crc16_buf[0];

			/* serialize/consume stop bit */
			read_continue_sddat0_lowspeed(ts_sdmmc_host, NULL, 0x0,
						      0x1, slot, false);

			spin_unlock_bh(&ts_sdmmc_host->bh_lock);

			/* check data crc16 */
			calc_crc16 = ts7800v1_sdmmc_crc16(pslot->rw_dma_buf,
							  data_blksz);

			if (calc_crc16 != sent_dat0_crc16) {
				dev_warn(
					mmc_dev(ts_sdmmc_host->mmc_host),
					"Warning, dat0 crc16 mismatch sent-crc16 %#x, calc-crc16 %#x\n",
					sent_dat0_crc16, calc_crc16);

				*data_error = ret = -EILSEQ;
				goto done;
			}
		}
	}

	*cmd_error = *data_error = ret = 0x0;

done:

	// 8 clocks before stopping
	spin_lock_bh(&ts_sdmmc_host->bh_lock);

	if (low_speed)
		for (i = 0; i < 8; i++) {
			/* toggle hi slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);
		}
	else {
		/* send 8-bits on fast clk-line */
		writeb(0xff, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
	}

	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	if (!IS_ERR_OR_NULL(dat0_sent_crc16_buf)) {
		kfree(dat0_sent_crc16_buf);
		dat0_sent_crc16_buf = NULL;
	}

	return ret;
}

static int send_cmd_recv_resp_write_blk(
	struct ts7800v1_sdmmc_host *ts_sdmmc_host, u8 slot, u32 cmd_opcode,
	u32 cmd_arg, unsigned int cmd_flags, int *cmd_error, u32 *cmd_resp,
	unsigned int data_blksz, u32 data_offset, int *data_error)
{
	struct ts7800v1_sdmmc_slot *pslot = &ts_sdmmc_host->sd_slot[slot];
	u32 blksz_dwords = data_blksz >> 2, x32;
	u8 stat, resp_len_bytes, sent_resp_crc7, calc_resp_crc7;
	bool low_speed, data_read_done;
	int ret = 0x0, i;

	/* initial ok state, following are not pre-set by default */
	pslot->sg_count = -1;
	pslot->cmd_timeout = 0x0;
	pslot->blk_buf_cycle_indx = pslot->blk_buf_nibble_indx = 0x0;
	data_read_done = false;
	/* clear crc_err/timeout */
	readb(ts_sdmmc_host->base_iomem + 0x1);

	/* low speed = sample on cmd-line, dat0-line */
	low_speed = (pslot->sd_state & SD_LOWSPEED) ? true : false;

	activate_slot_clk(ts_sdmmc_host, slot);

	if (IS_ERR_OR_NULL(pslot->rw_dma_buf)) {
		dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
			 "Error, No allocated DMA read buffer %ld\n",
			 PTR_ERR(pslot->rw_dma_buf));
		*data_error = ret = -ENOMEM;
		goto done;
	}

	if (!(cmd_flags & MMC_RSP_PRESENT)) {
		dev_warn(
			mmc_dev(ts_sdmmc_host->mmc_host),
			"Error, read block command flgas must have a response\n");
		*cmd_error = ret = -EINVAL;
	} else {
		switch ((cmd_flags &
			 (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			  MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
		case MMC_RSP_NONE:
			resp_len_bytes = 0x0;
			goto done;
		case MMC_RSP_R1:
		case MMC_RSP_R1B:
		case MMC_RSP_R3:
			resp_len_bytes = NORM_RESP_BYTES;
			break;
		case MMC_RSP_R2:
			resp_len_bytes = LONG_RESP_BYTES;
			break;
		default:
			dev_warn(mmc_dev(ts_sdmmc_host->mmc_host),
				 "Warning, Invalid response type\n");
			*cmd_error = ret = -EINVAL;
			goto done;
		}
	}

	/* serialize command */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	send_serialize_cmd(ts_sdmmc_host, pslot, cmd_opcode, cmd_arg,
			   low_speed);
	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	/* wait for response, i.e. start bit=0 on slow sd_cmd */
	mutex_lock(&ts_sdmmc_host->mutex_lock);
	wait_for_response(ts_sdmmc_host, pslot, low_speed);
	mutex_unlock(&ts_sdmmc_host->mutex_lock);

	if (pslot->cmd_timeout >= MAX_RESP_TIMEOUT_MICROSECS) {
		*cmd_error = ret = -ETIMEDOUT;
		goto done;
	}

	/* serialize/consume cmd response */
	spin_lock_bh(&ts_sdmmc_host->bh_lock);
	sd_cmd_read(ts_sdmmc_host, pslot, resp_len_bytes, sizeof(u32));

	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	/* serialize/consume card's busy response if any */
	if (cmd_flags & MMC_RSP_BUSY) {
		stat = 0x0;
		/* reset cmd time-out */
		pslot->cmd_timeout = 0x0;

		mutex_lock(&ts_sdmmc_host->mutex_lock);

		while ((stat & 0x7) != 0x7) {
			if (pslot->cmd_timeout++ >=
			    MAX_BUSY_TIMEOUT_MICROSECS) {
				*cmd_error = ret = -ETIMEDOUT;
				goto done;
			}

			/* toggle high slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			stat = stat << 1;
			stat |= readb(ts_sdmmc_host->base_iomem) & 0x1;

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);

			/* 1-delay reads */
			if (low_speed)
				add_1readb_delay(ts_sdmmc_host);

			usleep_range(15, 25);
		}

		mutex_unlock(&ts_sdmmc_host->mutex_lock);
	}

	/* process response outside of spin locks */
	if (cmd_flags & MMC_RSP_136) {
		/* R2, copy 12-bytes/3-double-words of argument including internal CRC7 */
		memcpy(&cmd_resp[0], &pslot->response[12], 0x4);
		memcpy(&cmd_resp[1], &pslot->response[8], 0x4);
		memcpy(&cmd_resp[2], &pslot->response[4], 0x4);
		memcpy(&cmd_resp[3], &pslot->response[0], 0x4);
	} else {
		/* R1, R1b, R3, R4, R5, R6 4-bytes arguments only*/
		memcpy(cmd_resp, &pslot->response[1], 0x4);
	}

	/* resp crc7 check */
	switch ((cmd_flags & (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
			      MMC_RSP_BUSY | MMC_RSP_OPCODE))) {
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
		sent_resp_crc7 = pslot->response[0] >> 1;
		calc_resp_crc7 = ts7800v1_sdmmc_crc7(
			0, &pslot->response[1], resp_len_bytes - 1, BE_ENDIAN);

		if (sent_resp_crc7 != calc_resp_crc7) {
			*cmd_error = ret = -EILSEQ;
			goto done;
		}
		break;
	default:
		/* no crc7 check*/
		break;
	}

	spin_lock_bh(&ts_sdmmc_host->bh_lock);

	add_2clk_cycles_high(ts_sdmmc_host);

	/* write start bit */
	writeb(CMDENB_DATENB_SDCLKH_SDCMDH_SDDAT03L, ts_sdmmc_host->base_iomem);
	/* 1-delay reads */
	add_1readb_delay(ts_sdmmc_host);

	writeb(CMDENB_DATENB_SDCLKL_SDCMDH_SDDAT03L, ts_sdmmc_host->base_iomem);
	/* 1-delay reads */
	add_1readb_delay(ts_sdmmc_host);

	for (i = 0; i < blksz_dwords; i++) {
		memcpy(&x32, &pslot->rw_dma_buf[i << 2], sizeof(u32));

		writel(x32, ts_sdmmc_host->base_iomem + SDCORE2_SDDATA_REG);
	}

	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	*cmd_error = *data_error = ret = 0x0;

done:

	// 8 clocks before stopping
	spin_lock_bh(&ts_sdmmc_host->bh_lock);

	if (low_speed)
		for (i = 0; i < 8; i++) {
			/* toggle hi slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);

			/* toggle low slow clk-line */
			writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
			       ts_sdmmc_host->base_iomem);
			/* 2-delay reads */
			add_2readb_delay(ts_sdmmc_host);
		}
	else {
		/* send 8-bits on fast clk-line */
		writeb(0xff, ts_sdmmc_host->base_iomem + SDCORE2_SDCMD_REG);
	}

	spin_unlock_bh(&ts_sdmmc_host->bh_lock);

	return ret;
}

static void ts7800v1_sdmmc_request(struct mmc_host *mmc,
				   struct mmc_request *mmc_req)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	struct mmc_command *req_cmd = mmc_req->cmd;
	struct mmc_command *sbc_cmd = mmc_req->sbc;
	struct mmc_command *stop_cmd = mmc_req->stop;

	if (req_cmd != NULL) {
		if (req_cmd->data != NULL &&
		    ((req_cmd->data->flags & MMC_DATA_READ) ||
		     (req_cmd->data->flags & MMC_DATA_WRITE))) {
			struct ts7800v1_sdmmc_slot *pslot =
				&ts_sdmmc_host->sd_slot[SD_ACTIVE_SLOT];
			u32 blocks;

			enum dma_data_direction dma_data_dir =
				mmc_get_dma_dir(req_cmd->data);
			if (dma_data_dir == DMA_NONE) {
				dev_err(mmc_dev(ts_sdmmc_host->mmc_host),
					"Invalid data direction %d\n",
					dma_data_dir);
				req_cmd->data->error = -EINVAL;
				goto mmc_request_done;
			}

			if (IS_ERR_OR_NULL(pslot)) {
				dev_err(mmc_dev(ts_sdmmc_host->mmc_host),
					"no valid slot selected!\n");
				req_cmd->error = -EINVAL;
				goto mmc_request_done;
			}

			pslot->sg_count = -1;
			blocks = (req_cmd->data->blksz * req_cmd->data->blocks);

			pslot->rw_dma_buf =
				kzalloc(blocks * sizeof(u8), GFP_KERNEL);
			if (IS_ERR_OR_NULL(pslot->rw_dma_buf)) {
				dev_err(mmc_dev(ts_sdmmc_host->mmc_host),
					"kzalloc failed with %ld\n",
					PTR_ERR(pslot->rw_dma_buf));
				req_cmd->data->error = -ENOMEM;
				goto done;
			}

			pslot->sg_count = dma_map_sg(
				mmc_dev(mmc_from_priv(ts_sdmmc_host)),
				req_cmd->data->sg, req_cmd->data->sg_len,
				dma_data_dir);

			if (pslot->sg_count == 0) {
				req_cmd->data->error = -ENOSPC;
				goto done;
			}

			req_cmd->data->sg_count = pslot->sg_count;

			if (req_cmd->data->flags & MMC_DATA_READ) {
				/* check request data state */

				if (sbc_cmd != NULL) {
					sbc_cmd->error =
						send_cmd_recv_resp_simple(
							ts_sdmmc_host,
							SD_ACTIVE_SLOT,
							sbc_cmd->opcode,
							sbc_cmd->arg,
							sbc_cmd->flags,
							&sbc_cmd->error,
							sbc_cmd->resp);
					if (sbc_cmd->error) {
						mmc_request_done(mmc, mmc_req);
						goto mmc_request_done;
					}
				}

				req_cmd->error = send_cmd_recv_resp_read_blk(
					ts_sdmmc_host, SD_ACTIVE_SLOT,
					req_cmd->opcode, req_cmd->arg,
					req_cmd->flags, &req_cmd->error,
					req_cmd->resp, blocks, 0,
					&req_cmd->data->error);

				/*
				 * fpga controller bug workaround, simply re-issue command
				 * This happens with some cards when querying their supported
				 * function groups prior to swtiching to high speed mode
				 */
				if (req_cmd->error && req_cmd->opcode == 0x6)
					req_cmd->error =
						send_cmd_recv_resp_read_blk(
							ts_sdmmc_host,
							SD_ACTIVE_SLOT,
							req_cmd->opcode,
							req_cmd->arg,
							req_cmd->flags,
							&req_cmd->error,
							req_cmd->resp, blocks,
							0,
							&req_cmd->data->error);

				if (req_cmd->error != 0x0 ||
				    req_cmd->data->error != 0x0)
					goto done;

				req_cmd->data->bytes_xfered += blocks;

				/* Send stop-transmission command if requested */
				if (stop_cmd != NULL && sbc_cmd == NULL) {
					stop_cmd->error =
						send_cmd_recv_resp_simple(
							ts_sdmmc_host,
							SD_ACTIVE_SLOT,
							stop_cmd->opcode,
							stop_cmd->arg,
							stop_cmd->flags,
							&stop_cmd->error,
							stop_cmd->resp);
				}

				/* mem-to-mem dma using SoC controller */
				sg_copy_from_buffer(req_cmd->data->sg,
						    req_cmd->data->sg_len,
						    pslot->rw_dma_buf, blocks);

			} else { /* MMC_DATA_WRITE */
				/* copy user data, mem-to-mem dma using SoC controller */
				sg_copy_to_buffer(req_cmd->data->sg,
						  req_cmd->data->sg_len,
						  pslot->rw_dma_buf, blocks);

				/* check request data state */

				if (sbc_cmd != NULL) {
					sbc_cmd->error =
						send_cmd_recv_resp_simple(
							ts_sdmmc_host,
							SD_ACTIVE_SLOT,
							sbc_cmd->opcode,
							sbc_cmd->arg,
							sbc_cmd->flags,
							&sbc_cmd->error,
							sbc_cmd->resp);
					if (sbc_cmd->error) {
						mmc_request_done(mmc, mmc_req);
						goto mmc_request_done;
					}
				}

				req_cmd->error = send_cmd_recv_resp_write_blk(
					ts_sdmmc_host, SD_ACTIVE_SLOT,
					req_cmd->opcode, req_cmd->arg,
					req_cmd->flags, &req_cmd->error,
					req_cmd->resp, blocks, 0,
					&req_cmd->data->error);

				if (req_cmd->error != 0x0 ||
				    req_cmd->data->error != 0x0)
					goto done;

				req_cmd->data->bytes_xfered += blocks;

				/* Send stop-transmission command if requested */
				if (stop_cmd != NULL && sbc_cmd == NULL) {
					stop_cmd->error =
						send_cmd_recv_resp_simple(
							ts_sdmmc_host,
							SD_ACTIVE_SLOT,
							stop_cmd->opcode,
							stop_cmd->arg,
							stop_cmd->flags,
							&stop_cmd->error,
							stop_cmd->resp);
				}
			}

		done:
			if (pslot->sg_count > 0) {
				dma_unmap_sg(
					mmc_dev(mmc_from_priv(ts_sdmmc_host)),
					req_cmd->data->sg,
					req_cmd->data->sg_len, dma_data_dir);
				pslot->sg_count = -1;
			}

			if (!IS_ERR_OR_NULL(pslot->rw_dma_buf)) {
				kfree(pslot->rw_dma_buf);
				pslot->rw_dma_buf = NULL;
			}

		} else {
			req_cmd->error = send_cmd_recv_resp_simple(
				ts_sdmmc_host, SD_ACTIVE_SLOT, req_cmd->opcode,
				req_cmd->arg, req_cmd->flags, &req_cmd->error,
				req_cmd->resp);

			/* fpga controller bug workaround, simply re-issue command
			* This happens with some cards when querying their supported
			* function groups prior to swtiching to high speed mode
			*/
			if (req_cmd->error && req_cmd->opcode == 0x6)
				req_cmd->error = send_cmd_recv_resp_simple(
					ts_sdmmc_host, SD_ACTIVE_SLOT,
					req_cmd->opcode, req_cmd->arg,
					req_cmd->flags, &req_cmd->error,
					req_cmd->resp);
		}
	}

mmc_request_done:
	mmc_request_done(mmc, mmc_req);
}

static void ts7800v1_sdmmc_host_init(struct ts7800v1_sdmmc_host *ts_sdmmc_host)
{
	ts_sdmmc_host->hw_version = ts7800v1_sdmmc_hw_version(ts_sdmmc_host);
	card_reset(ts_sdmmc_host, SD_ACTIVE_SLOT);
}

static void ts7800v1_sdmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	/* change
	 * power supply mode,
	 * data bus width,
	 * timing specification used, such as: MMC_TIMING_LEGACY, MMC_TIMING_MMC_HS, MMC_TIMING_UHS_SDR12, etc
	 * signalling voltage (1.8V or 3.3V),
	 * driver type (A, B, C, D)
	 */

	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	struct ts7800v1_sdmmc_slot *pslot =
		&ts_sdmmc_host->sd_slot[SD_ACTIVE_SLOT];

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		set_clkspd(ts_sdmmc_host, pslot, false /*slow*/);
		break;

	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_SD_HS:
	default:
		set_clkspd(ts_sdmmc_host, pslot, true /*fast*/);
	}

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_4:
		pslot->sd_state |= DATSSP_4BIT;
		set_mlt_rdwr(ts_sdmmc_host, pslot, true /* multi */);

		break;
	default:
		/* keep default 1 bit mode */
		pslot->sd_state &= ~DATSSP_4BIT;
		set_mlt_rdwr(ts_sdmmc_host, pslot, false);
	}
}

static int ts7800v1_sdmmc_card_busy(struct mmc_host *mmc)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	u8 stat, a, i;

	stat = 0x0;
	for (i = 0; i < 8; i++) {
		/* toggle high slow clk-line */
		writeb(CMDTRI_DATTRI_SDCLKH_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* 1-delay reads */
		a = readb(ts_sdmmc_host->base_iomem);

		/* look for 3-consecutive dat0 = 1 */
		stat = stat << 1;
		/* check dat0 for busy bit=1 */
		stat |= readb(ts_sdmmc_host->base_iomem) & 0x1;

		writeb(CMDTRI_DATTRI_SDCLKL_SDCMDH_SDDATH,
		       ts_sdmmc_host->base_iomem);

		/* 1-delay reads */
		a = readb(ts_sdmmc_host->base_iomem);

		if ((stat & 0x7) == 0x7)
			break;
	}

	stat &= GENMASK(3, 0);
	return !(stat == GENMASK(3, 0));
}

static void ts7800v1_sdmmc_hw_reset(struct mmc_host *mmc)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	card_reset(ts_sdmmc_host, SD_ACTIVE_SLOT);
}

/*
 * 0 for a read/write card
 * 1 for a read-only card
 */
static int ts7800v1_sdmmc_get_ro(struct mmc_host *mmc)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	return ts_sdmmc_host->sd_slot[SD_ACTIVE_SLOT].sd_wprot;
}

/*
 * 0 for a absent card
 * 1 for a present card
 */
static int ts7800v1_sdmmc_get_cd(struct mmc_host *mmc)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = mmc_priv(mmc);
	return ts_sdmmc_host->sd_slot[SD_ACTIVE_SLOT].sd_detect;
}

static const struct mmc_host_ops ts7800v1_sdmmc_host_ops = {
	.request = ts7800v1_sdmmc_request,
	.set_ios = ts7800v1_sdmmc_set_ios,
	.get_ro = ts7800v1_sdmmc_get_ro,
	.get_cd = ts7800v1_sdmmc_get_cd,
	.card_busy = ts7800v1_sdmmc_card_busy,
	.hw_reset = ts7800v1_sdmmc_hw_reset,
};

static int ts7800v1_sdmmc_probe(struct platform_device *pdev)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = NULL;
	struct mmc_host *mmc_host = NULL;
	struct resource *mem_res = NULL, *irq_res = NULL;
	int ret;

	mmc_host =
		mmc_alloc_host(sizeof(struct ts7800v1_sdmmc_host), &pdev->dev);
	if (IS_ERR_OR_NULL(mmc_host)) {
		dev_err(&pdev->dev, "Failed to allocate mmc host, error %ld \n",
			PTR_ERR(mmc_host));
		ret = PTR_ERR(mmc_host);
		goto err_alloc_host;
	}

	ts_sdmmc_host = mmc_priv(mmc_host);
	ts_sdmmc_host->mmc_host = mmc_host;
	ts_sdmmc_host->base_iomem = NULL;

	spin_lock_init(&ts_sdmmc_host->bh_lock);
	mutex_init(&ts_sdmmc_host->mutex_lock);

	mem_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					       "ts_sdmmc_ctrl");
	if (IS_ERR_OR_NULL(mem_res)) {
		dev_err(&pdev->dev,
			"Failed to get platform memory resource, error %ld \n",
			PTR_ERR(mem_res));
		ret = PTR_ERR(mem_res);
		goto pltfrm_get_res_mem_err;
	}

	ts_sdmmc_host->base_iomem = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR_OR_NULL(ts_sdmmc_host->base_iomem)) {
		dev_err(&pdev->dev,
			"Failed to IO map resource %s, error %ld \n",
			mem_res->name, PTR_ERR(ts_sdmmc_host->base_iomem));
		ret = PTR_ERR(ts_sdmmc_host->base_iomem);
		goto devm_ioremap_res_mem_err;
	}

	irq_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					       "ts_sdmmc_sdbusy");
	if (IS_ERR_OR_NULL(irq_res)) {
		dev_err(&pdev->dev, "Failed to get irq resource, error %ld \n",
			PTR_ERR(irq_res));
		ret = PTR_ERR(irq_res);
		goto pltfrm_get_res_irq_err;
	}

	/* ensure 4-bit bus_width (only width supported by hardware) */
	mmc_host->caps &= ~MMC_CAP_8_BIT_DATA;
	mmc_host->caps |= MMC_CAP_4_BIT_DATA;

	/* controller does not auto-generate CMD23 */
	mmc_host->caps &= ~MMC_CAP_CMD23;

	mmc_host->max_blk_count = MAX_BLK_COUNT;
	mmc_host->max_blk_size = MAX_BLK_SIZE;
	mmc_host->max_req_size =
		mmc_host->max_blk_count * mmc_host->max_blk_size;
	mmc_host->max_segs = MAX_SEGS;
	mmc_host->max_seg_size = MAX_SEG_SIZE;

	/* Set default capabilities */
	mmc_host->caps |= MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_MMC_HIGHSPEED |
			  MMC_CAP_SD_HIGHSPEED;

	mmc_host->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc_host->ops = &ts7800v1_sdmmc_host_ops;

	ts7800v1_sdmmc_host_init(ts_sdmmc_host);

	platform_set_drvdata(pdev, ts_sdmmc_host);

	ret = mmc_add_host(mmc_host);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"Failed to add TS-7800v1 SD/MMC host controller, error %d \n",
			ret);
		goto err_mmc_add_host;
	}

	dev_info(&pdev->dev,
		 "TS-7800v1 FPGA based SD/MMC Controller initialized\n");

	return 0;

err_mmc_add_host:
	devm_free_irq(&pdev->dev, ts_sdmmc_host->sdbusy_irq, ts_sdmmc_host);
pltfrm_get_res_irq_err:
	devm_iounmap(&pdev->dev, ts_sdmmc_host->base_iomem);
devm_ioremap_res_mem_err:
pltfrm_get_res_mem_err:
	mutex_destroy(&ts_sdmmc_host->mutex_lock);
	mmc_free_host(mmc_host);
err_alloc_host:
	return ret;
}

static int ts7800v1_sdmmc_remove(struct platform_device *pdev)
{
	struct ts7800v1_sdmmc_host *ts_sdmmc_host = platform_get_drvdata(pdev);

	mmc_remove_host(ts_sdmmc_host->mmc_host);
	//ts7800v1_sdmmc_release_dma_channels(host);
	mutex_destroy(&ts_sdmmc_host->mutex_lock);
	if (!IS_ERR_OR_NULL(ts_sdmmc_host->mmc_host))
		mmc_free_host(ts_sdmmc_host->mmc_host);

	dev_info(&pdev->dev,
		 "TS-7800v1 FPGA based SD/MMC controller removed\n");

	return 0;
}

static const struct platform_device_id ts7800v1_sdmmc_ids[] = {
	{
		.name = DRIVER_NAME,
	},
	{
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(platform, ts7800v1_sdmmc_ids);

static struct platform_driver ts7800v1_sdmmc_driver = {
	.probe = ts7800v1_sdmmc_probe,
	.remove = ts7800v1_sdmmc_remove,
	.id_table	= ts7800v1_sdmmc_ids,
	.driver = {
		.name = DRIVER_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

module_platform_driver(ts7800v1_sdmmc_driver);

MODULE_DESCRIPTION("TS-7800v1 FPGA based MMC/SD Driver");
MODULE_AUTHOR("Firas Ashkar <firas.ashkar@savoirfairelinux.com>");
MODULE_LICENSE("GPL v2");
