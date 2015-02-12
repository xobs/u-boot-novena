/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <i2c.h>
#include <ddr_spd.h>

/* Required because mx6-ddr.h doesn't include this when building for SPL */
#ifdef CONFIG_MX6Q
#include <asm/arch/mx6q-ddr.h>
#else
#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#include <asm/arch/mx6dl-ddr.h>
#else
#ifdef CONFIG_MX6SX
#include <asm/arch/mx6sx-ddr.h>
#else
#error "Please select cpu"
#endif  /* CONFIG_MX6SX */
#endif  /* CONFIG_MX6DL or CONFIG_MX6S */
#endif  /* CONFIG_MX6Q */

#ifndef CONFIG_SPL_I2C_SUPPORT
#error "No SPL I2C support"
#endif

#define CONFIG_PERFORM_RAM_TEST

DECLARE_GLOBAL_DATA_PTR;

#define MPDGCTRL0_PHY0 0x021b083c
#define MPDGCTRL1_PHY0 0x021b0840
#define MPDGCTRL0_PHY1 0x021b483c
#define MPDGCTRL1_PHY1 0x021b4840
#define MPRDDLCTL_PHY0 0x021b0848
#define MPRDDLCTL_PHY1 0x021b4848
#define MPWRDLCTL_PHY0 0x021b0850
#define MPWRDLCTL_PHY1 0x021b4850

#define MDPDC 0x0004
#define MDCFG0 0x000C
#define MDCFG1 0x0010
#define MDCFG2 0x0014
#define MAPSR 0x0404
#define MDREF 0x0020
#define MDASP 0x0040
#define MPZQHWCTRL 0x0800
#define MDSCR 0x001C
#define MPWLGCR 0x0808
#define MPWLDECTRL0 0x080c
#define MPWLDECTRL1 0x0810
#define MDCTL 0x0000
#define MDMISC 0x0018
#define MPPDCMPR1 0x088C
#define MPSWDAR 0x0894
#define MPRDDLCTL 0x0848
#define MPMUR 0x08B8
#define MPDGCTRL0 0x083C
#define MPDGHWST0 0x087C
#define MPDGHWST1 0x0880
#define MPDGHWST2 0x0884
#define MPDGHWST3 0x0888
#define MPDGCTRL1 0x0840
#define MPRDDLHWCTL 0x0860
#define MPWRDLCTL 0x0850
#define MPWRDLHWCTL 0x0864

#define GET_BIT(n,x) ( ( (x) >> (n) ) & 1 )

static inline void mmdcp0_setbit(uint32_t bit, uint32_t offset)
{
	*((volatile uint32_t *)(MMDC_P0_BASE_ADDR + offset)) |= (1 << bit);
}

static inline void mmdcp1_setbit(uint32_t bit, uint32_t offset)
{
	*((volatile uint32_t *)(MMDC_P1_BASE_ADDR + offset)) |= (1 << bit);
}

static inline void mmdcp0_clrbit(uint32_t bit, uint32_t offset)
{
	*((volatile uint32_t *)(MMDC_P0_BASE_ADDR + offset)) &= ~(1 << bit);
}

static inline uint32_t read_mmdcp0(uint32_t offset)
{
	return readl(MMDC_P0_BASE_ADDR + offset);
}

static inline uint32_t read_mmdcp1(uint32_t offset)
{
	return readl(MMDC_P1_BASE_ADDR + offset);
}

static inline void write_mmdcp0(uint32_t value, uint32_t offset)
{
	writel(value, MMDC_P0_BASE_ADDR + offset);
}

static inline void write_mmdcp1(uint32_t value, uint32_t offset)
{
	writel(value, MMDC_P1_BASE_ADDR + offset);
}

static void dram_fatal(uint8_t arg)
{
	printf("Fatal DDR error 0x%02x\r\n", arg);
	while(1);
}

#ifdef CONFIG_PERFORM_RAM_TEST
static int write_random_data(int bank)
{
	ulong	addr, writeval, count, feedback, sum1;
	int	size;
	ulong	base_address = 0x10000000;

	if (bank == 0)
		addr = base_address;
	else
		addr = base_address + 0x90000000;
	
	/* Get the value to write.  */
	writeval = 0x0;

	count = 0x20000;
	size = 2;
	sum1 = 0;

	debug("16-bit writes @ 0x%08lx - 0x%08lx / %u: ",
		addr, addr + (count * sizeof(addr)*size), sizeof(addr) * size);

	while (count-- > 0) {
		feedback = (GET_BIT(14, writeval) == GET_BIT(13, writeval));
		writeval = (writeval << 1) + feedback;
		writeval &= 0x7FFF;
		*((ushort  *)addr) = (ushort )writeval;
		sum1 += (ushort )writeval;
		addr += size;
	}
	debug("checksum: %08lx\n", sum1);
	return 0;
}

int read_random_data(int bank)
{
	ulong	addr, writeval, count, feedback, sum1, sum2;
	int	size;
	ulong	base_address = 0x10000000;

	/* Address is specified since argc > 1 */
	if (bank == 0)
		addr = base_address;
	else
		addr = base_address + 0x90000000;

	/* Get the value to write.  */
	writeval = 0x0;
	size = 2;

	sum1 = 0;
	sum2 = 0;
	count = 0x20000;
	feedback = 0;

	debug("16-bit reads @ 0x%08lx - 0x%08lx / %u: ",
		addr, addr + (count * sizeof(addr)*size), sizeof(addr) * size);

	while (count-- > 0) {
		feedback = (GET_BIT(14, writeval) == GET_BIT(13, writeval));
		writeval = (writeval << 1) + feedback;
		writeval &= 0x7FFF;
		sum1 += (ushort )writeval;
		sum2 += *((ushort  *)addr);
		addr += size;
	}
	debug("computed: %08lx, readback: %08lx\n", sum1, sum2);
	return sum1 != sum2;
}
#endif /* CONFNIG_PERFORM_RAM_TEST */

static void reset_read_data_fifos(void)
{
	 /* Read data FIFOs reset1.  */
	mmdcp0_setbit(31, MPDGCTRL0);
	while (read_mmdcp0(MPDGCTRL0) & 0x80000000)
		debug("1");

	/* Read data FIFOs reset2 */
	mmdcp0_setbit(31, MPDGCTRL0);
	while (read_mmdcp0(MPDGCTRL0) & 0x80000000)
		debug("2");
}

static void precharge_all(int cs0_enable, int cs1_enable)
{
	/*
	 * Issue the Precharge-All command to the DDR device for both
	 * chip selects.  Note, CON_REQ bit should also remain set.
	 * If only using one chip select, then precharge only the desired
	 * chip select.
	 */
	if (cs0_enable == 1) { /* CS0 */
		write_mmdcp0(0x04008050, MDSCR);
		while (!(read_mmdcp0(MDSCR) & 0x4000))
			debug("x");
	}

	if (cs1_enable == 1) { /* CS1 */
		write_mmdcp0(0x04008058, MDSCR);
		while (!(read_mmdcp0(MDSCR) & 0x4000))
			debug("y");
	}
}

static void force_delay_measurement(int bus_size)
{
	write_mmdcp0(0x800, MPMUR);
	if (bus_size == 0x2)
		write_mmdcp1(0x800, MPMUR);
}

static void modify_dg_result(u32 reg_st0, u32 reg_st1, u32 reg_ctrl)
{
	u32 dg_tmp_val, dg_dl_abs_offset, dg_hc_del, val_ctrl;

	/*
	 * DQS gating absolute offset should be modified from reflecting
	 *(HW_DG_LOWx + HW_DG_UPx)/2 to reflecting (HW_DG_UPx - 0x80)
	 */

	val_ctrl = readl(reg_ctrl);
	val_ctrl &= 0xf0000000;

	dg_tmp_val = ((readl(reg_st0) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= dg_dl_abs_offset + dg_hc_del;

	dg_tmp_val = ((readl(reg_st1) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= (dg_dl_abs_offset + dg_hc_del) << 16;

	writel(val_ctrl, reg_ctrl);
}

int do_write_level_calibration(void)
{
	int esdmisc_val, zq_val;
	int errors = 0;
	int ldectrl[4];
	uint32_t ddr_mr1 = 0x4;

	/*
	 * Stash old values in case calibration fails,
	 * we need to restore them
	 */
	ldectrl[0] = read_mmdcp0(MPWLDECTRL0);
	ldectrl[1] = read_mmdcp0(MPWLDECTRL1);
	ldectrl[2] = read_mmdcp1(MPWLDECTRL0);
	ldectrl[3] = read_mmdcp1(MPWLDECTRL1);

	/* disable DDR logic power down timer */
	write_mmdcp0(read_mmdcp0(MDPDC) & 0xffff00ff, MDPDC);

	/* disable Adopt power down timer */
	write_mmdcp0(read_mmdcp0(MAPSR) | 0x1, MAPSR);

	debug("Start write leveling calibration");

	/*
	 * 2. disable auto refresh and ZQ calibration
	 * before proceeding with Write Leveling calibration
	 */
	esdmisc_val = read_mmdcp0(MDREF);
	write_mmdcp0(0x0000C000, MDREF);
	zq_val = read_mmdcp0(MPZQHWCTRL);
	write_mmdcp0(zq_val & ~(0x3), MPZQHWCTRL);

	/* 3. increase walat and ralat to maximum */
	mmdcp0_setbit(6, MDMISC); //set RALAT to max
	mmdcp0_setbit(7, MDMISC);
	mmdcp0_setbit(8, MDMISC);
	mmdcp0_setbit(16, MDMISC); //set WALAT to max
	mmdcp0_setbit(17, MDMISC);

	mmdcp1_setbit(6, MDMISC); //set RALAT to max
	mmdcp1_setbit(7, MDMISC);
	mmdcp1_setbit(8, MDMISC);
	mmdcp1_setbit(16, MDMISC); //set WALAT to max
	mmdcp1_setbit(17, MDMISC);

	/*
	 * 4 & 5. Configure the external DDR device to enter write-leveling
	 * mode through Load Mode Register command.
	 * Register setting:
	 * Bits[31:16] MR1 value (0x0080 write leveling enable)
	 * Bit[9] set WL_EN to enable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	write_mmdcp0(0x00808231, MDSCR);

	/* 6. Activate automatic calibration by setting MPWLGCR[HW_WL_EN] */
	write_mmdcp0(0x00000001, MPWLGCR);

	/*
	 * 7. Upon completion of this process the MMDC de-asserts
	 * the MPWLGCR[HW_WL_EN]
	 */
	while (read_mmdcp0(MPWLGCR) & 0x00000001)
		debug(".");

	/*
	 * 8. check for any errors: check both PHYs for x64 configuration,
	 * if x32, check only PHY0
	 */
	if (read_mmdcp0(MPWLGCR) & 0x00000F00)
		errors |= 1;
	if (read_mmdcp1(MPWLGCR) & 0x00000F00)
		errors |= 2;
	debug(" Done.  Error mask: 0x%x\n", errors);

	/* check to see if cal failed */
	if ((read_mmdcp0(MPWLDECTRL0)) == 0x001F001F
	 && (read_mmdcp0(MPWLDECTRL1)) == 0x001F001F
	 && (read_mmdcp1(MPWLDECTRL0)) == 0x001F001F
	 && (read_mmdcp1(MPWLDECTRL1)) == 0x001F001F) {
		debug("Cal seems to have soft-failed due to memory "
			"not supporting write leveling on all channels. "
			"Restoring original write leveling values.\n");
		write_mmdcp0(ldectrl[0], MPWLDECTRL0);
		write_mmdcp0(ldectrl[1], MPWLDECTRL1);
		write_mmdcp1(ldectrl[2], MPWLDECTRL0);
		write_mmdcp1(ldectrl[3], MPWLDECTRL1);
		errors |= 4;
	}

	/*
	 * User should issue MRS command to exit write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value "ddr_mr1" value from initialization
	 * Bit[9] clear WL_EN to disable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	write_mmdcp0((ddr_mr1 << 16) + 0x8031, MDSCR);

	/* re-enable auto refresh and zq cal */
	write_mmdcp0(esdmisc_val, MDREF);
	write_mmdcp0(zq_val, MPZQHWCTRL);

#ifdef DEBUG
	debug("\tMMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
		read_mmdcp0(MPWLDECTRL0));
	debug("\tMMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
		read_mmdcp0(MPWLDECTRL1));
	debug("\tMMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
		read_mmdcp1(MPWLDECTRL0));
	debug("\tMMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
		read_mmdcp1(MPWLDECTRL1));
#else
	/* We must force a readback of these values, to get them to stick */
	read_mmdcp0(MPWLDECTRL0);
	read_mmdcp0(MPWLDECTRL1);
	read_mmdcp1(MPWLDECTRL0);
	read_mmdcp1(MPWLDECTRL1);
#endif
	/* enable DDR logic power down timer: */
	write_mmdcp0(read_mmdcp0((MDPDC)) | 0x00005500, MDPDC);

	/* Enable Adopt power down timer: */
	write_mmdcp0(read_mmdcp0((MAPSR)) & 0xfffffff7, MAPSR);

	/* Clear CON_REQ */
	write_mmdcp0(0, MDSCR);

	return errors;
}

int do_dqs_calibration(void)
{
	int esdmisc_val;
	int bus_size;
	int temp_ref;
	int cs0_enable;
	int cs1_enable;
	int cs0_enable_initial;
	int cs1_enable_initial;
	int PDDWord = 0x00ffff00; /* best so far, place into MPPDCMPR1 */
	int errors = 0;
	unsigned int initdelay = 0x40404040;

	/* check to see which chip selects are enabled */
	cs0_enable_initial = (read_mmdcp0(MDCTL) & 0x80000000) >> 31;
	cs1_enable_initial = (read_mmdcp0(MDCTL) & 0x40000000) >> 30;

	/* disable DDR logic power down timer: */
	write_mmdcp0(read_mmdcp0(MDPDC) & 0xffff00ff, MDPDC);

	/* disable Adopt power down timer: */
	mmdcp0_setbit(1, MAPSR);

	/* set DQS pull ups */
	writel(readl(MX6_IOM_DRAM_SDQS0) | 0x7000, MX6_IOM_DRAM_SDQS0);
	writel(readl(MX6_IOM_DRAM_SDQS1) | 0x7000, MX6_IOM_DRAM_SDQS1);
	writel(readl(MX6_IOM_DRAM_SDQS2) | 0x7000, MX6_IOM_DRAM_SDQS2);
	writel(readl(MX6_IOM_DRAM_SDQS3) | 0x7000, MX6_IOM_DRAM_SDQS3);
	writel(readl(MX6_IOM_DRAM_SDQS4) | 0x7000, MX6_IOM_DRAM_SDQS4);
	writel(readl(MX6_IOM_DRAM_SDQS5) | 0x7000, MX6_IOM_DRAM_SDQS5);
	writel(readl(MX6_IOM_DRAM_SDQS6) | 0x7000, MX6_IOM_DRAM_SDQS6);
	writel(readl(MX6_IOM_DRAM_SDQS7) | 0x7000, MX6_IOM_DRAM_SDQS7);

	/* Save old RALAT and WALAT values */
	esdmisc_val = read_mmdcp0(MDMISC);

	mmdcp0_setbit(6, MDMISC); //set RALAT to max
	mmdcp0_setbit(7, MDMISC);
	mmdcp0_setbit(8, MDMISC);
	mmdcp0_setbit(16, MDMISC); //set WALAT to max
	mmdcp0_setbit(17, MDMISC);

	/* Disable auto refresh before proceeding with calibration */
	temp_ref = read_mmdcp0(MDREF);
	write_mmdcp0(0x0000c000, MDREF);

	/*
	 * Per the ref manual, issue one refresh cycle MDSCR[CMD]= 0x2,
	 * this also sets the CON_REQ bit.
	 */
	if (cs0_enable_initial == 1)
		write_mmdcp0(0x00008020, MDSCR);
	if (cs1_enable_initial == 1)
		write_mmdcp0(0x00008028, MDSCR);

	/* poll to make sure the con_ack bit was asserted */
	while (!(read_mmdcp0(MDSCR) & 0x00004000))
		debug("c");

	/*
	 * Check MDMISC register CALIB_PER_CS to see which CS calibration
	 * is targeted to (under normal cases, it should be cleared
	 * as this is the default value, indicating calibration is directed
	 * to CS0).
	 * Disable the other chip select not being target for calibration
	 * to avoid any potential issues.  This will get re-enabled at end
	 * of calibration.
	 */
	if ((read_mmdcp0(MDMISC) & 0x00100000) == 0)
		/* clear SDE_1 */
		mmdcp0_clrbit(30, MDCTL);
	else
		/* clear SDE_0 */
		mmdcp0_clrbit(31, MDCTL);

	/*
	 * Check to see which chip selects are now enabled for
	 * the remainder of the calibration.
	 */
	cs0_enable = (read_mmdcp0(MDCTL) & 0x80000000) >> 31;
	cs1_enable = (read_mmdcp0(MDCTL) & 0x40000000) >> 30;

	/* Check to see what the data bus size is */
	bus_size = (read_mmdcp0(MDCTL) & 0x30000) >> 16;
	debug("Data bus size: %d (%d bits)\n", bus_size, 1 << (bus_size + 4));

	precharge_all(cs0_enable, cs1_enable);

	/* Write the pre-defined value into MPPDCMPR1 */
	write_mmdcp0(PDDWord, MPPDCMPR1);

	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll
	 * this bit until it clears to indicate completion of the write access.
	 */
	mmdcp0_setbit(0, MPSWDAR);
	while (read_mmdcp0(MPSWDAR) & 0x00000001)
		debug(".");

	/* Set the RD_DL_ABS# bits to their default values
	 * (will be calibrated later in the read delay-line calibration).
	 * Both PHYs for x64 configuration, if x32, do only PHY0.
	 */
	write_mmdcp0(initdelay, MPRDDLCTL);
	if (bus_size == 0x2)
		write_mmdcp1(initdelay, MPRDDLCTL);

	/* Force a measurment, for previous delay setup to take effect. */
	force_delay_measurement(bus_size);

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Read DQS Gating calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	debug("Starting Read DQS Gating calibration [");

	/*
	 * Reset the read data FIFOs (two resets); only need to issue reset
	 * to PHY0 since in x64 mode, the reset will also go to PHY1.
	 */
	reset_read_data_fifos();

	/*
	 * Start the automatic read DQS gating calibration process by
	 * asserting MPDGCTRL0[HW_DG_EN] and MPDGCTRL0[DG_CMP_CYC]
	 * and then poll MPDGCTRL0[HW_DG_EN]] until this bit clears
	 * to indicate completion.
	 * Also, ensure that MPDGCTRL0[HW_DG_ERR] is clear to indicate
	 * no errors were seen during calibration.
	 */

	/*
	 * Set bit 30: chooses option to wait 32 cycles instead of
	 * 16 before comparing read data.
	 */
	mmdcp0_setbit(30, MPDGCTRL0);

	/* Set bit 28 to start automatic read DQS gating calibration */
	mmdcp0_setbit(28, MPDGCTRL0);

	/* Poll for completion.  MPDGCTRL0[HW_DG_EN] should be 0 */
	while (read_mmdcp0(MPDGCTRL0) & 0x10000000)
		debug(".");

	/*
	 * Check to see if any errors were encountered during calibration
	 * (check MPDGCTRL0[HW_DG_ERR]).
	 * Check both PHYs for x64 configuration, if x32, check only PHY0.
	 */
	if (read_mmdcp0(MPDGCTRL0) & 0x00001000) {
		debug("0");
		errors |= 1;
	}
	if ((bus_size == 0x2) && (read_mmdcp1(MPDGCTRL0) & 0x00001000)) {
		debug("1");
		errors |= 2;
	}
	
	/*
	 * DQS gating absolute offset should be modified from
	 * reflecting (HW_DG_LOWx + HW_DG_UPx)/2 to
	 * reflecting (HW_DG_UPx - 0x80)
	 */
	modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST0,
		MMDC_P0_BASE_ADDR + MPDGHWST1,
		MMDC_P0_BASE_ADDR + MPDGCTRL0);
	modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST2,
		MMDC_P0_BASE_ADDR + MPDGHWST3,
		MMDC_P0_BASE_ADDR + MPDGCTRL1);
	if (bus_size == 0x2) {
		modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST0),
			(MMDC_P1_BASE_ADDR + MPDGHWST1),
			(MMDC_P1_BASE_ADDR + MPDGCTRL0));
		modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST2),
			(MMDC_P1_BASE_ADDR + MPDGHWST3),
			(MMDC_P1_BASE_ADDR + MPDGCTRL1));
	}
	debug("] Done.  Error mask: 0x%x.\n", errors);

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Read delay Calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	debug("Starting Read Delay calibration [");

	reset_read_data_fifos();
	
	/*
	 * 4. Issue the Precharge-All command to the DDR device for both
	 * chip selects.  If only using one chip select, then precharge
	 * only the desired chip select.
	 */
	precharge_all(cs0_enable, cs1_enable);


	/* *********** 5. 6. 7. set the pre-defined word ************ */
	write_mmdcp0(PDDWord, MPPDCMPR1);

	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll
	 * this bit until it clears to indicate completion of the write access.
	 */
	mmdcp0_setbit(0, MPSWDAR);
	while (read_mmdcp0(MPSWDAR) & 0x00000001)
		debug("d");

	/* 8. set initial delays to center up dq in clock */
	write_mmdcp0(initdelay, MPRDDLCTL);
	if (bus_size == 0x2)
		write_mmdcp1(initdelay, MPRDDLCTL);
//	debug("initial delay 0: %08x / initial delay 1: %08x\n", 
//		read_mmdcp0(MPRDDLCTL), read_mmdcp1(MPRDDLCTL));

	/*
	 * 9. Read delay-line calibration
	 * Start the automatic read calibration process by asserting
	 * MPRDDLHWCTL[HW_RD_DL_EN].
	 */
	write_mmdcp0(0x00000030, MPRDDLHWCTL);

	/*
	 * 10. poll for completion
	 * MMDC indicates that the write data calibration had finished by
	 * setting MPRDDLHWCTL[HW_RD_DL_EN] = 0.   Also, ensure that
	 * no error bits were set.
	 */
	while (read_mmdcp0(MPRDDLHWCTL) & 0x00000010)
		debug("p");

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (read_mmdcp0(MPRDDLHWCTL) & 0x0000000f) {
		debug("0");
		errors |= 4;
	}
	if ((bus_size == 0x2) && (read_mmdcp1(MPRDDLHWCTL) & 0x0000000f)) {
		debug("1");
		errors |= 8;
	}
	debug("] Done.  Error mask: 0x%x.\n", errors);

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Write delay Calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	debug("Starting Write Delay calibration [");

	reset_read_data_fifos();

	/*
	 * 4. Issue the Precharge-All command to the DDR device for both
	 * chip selects. If only using one chip select, then precharge
	 * only the desired chip select.
	 */
	precharge_all(cs0_enable, cs1_enable);

#if 0
	/* *********** 5. 6. 7. set the pre-defined word ************ */
	write_mmdcp0(PDDWord, MPPDCMPR1);

	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll this bit
	 * until it clears to indicate completion of the write access.
	 */
	mmdcp0_setbit(0, MPSWDAR);
	while (read_mmdcp0(MPSWDAR) & 0x00000001)
		debug("w");
#endif

	/*
	 * 8. Set the WR_DL_ABS# bits to their default values.
	 * Both PHYs for x64 configuration, if x32, do only PHY0.
	 */
	write_mmdcp0(initdelay, MPWRDLCTL);
	if (bus_size == 0x2)
		write_mmdcp1(initdelay, MPWRDLCTL);

	/*
	 * XXX This isn't in the manual. Force a measurment,
	 * for previous delay setup to effect.
	 */
	force_delay_measurement(bus_size);

	/*
	 * 9. 10. Start the automatic write calibration process
	 * by asserting MPWRDLHWCTL0[HW_WR_DL_EN].
	 */
	write_mmdcp0(0x00000030, MPWRDLHWCTL);

	/*
	 * Poll for completion.
	 * MMDC indicates that the write data calibration had finished
	 * by setting MPWRDLHWCTL[HW_WR_DL_EN] = 0.
	 * Also, ensure that no error bits were set.
	 */
	while (read_mmdcp0(MPWRDLHWCTL) & 0x00000010)
		debug("p");

	/* Check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (read_mmdcp0(MPWRDLHWCTL) & 0x0000000f) {
		debug("0");
		errors |= 16;
	}
	if ((bus_size == 0x2) && (read_mmdcp1(MPWRDLHWCTL) & 0x0000000f)) {
		debug("1");
		errors |= 32;
	}

	debug("] Done.  Error mask: 0x%x.\n", errors);

	reset_read_data_fifos();

	/* Enable DDR logic power down timer: */
	write_mmdcp0(read_mmdcp0(MDPDC) | 0x00005500, MDPDC);

	/* Enable Adopt power down timer: */
	write_mmdcp0(read_mmdcp0(MAPSR) & 0xfffffff7, MAPSR);

	/* Restore MDMISC value (RALAT, WALAT) to MMDCP1 */
	write_mmdcp1(esdmisc_val, MDMISC);

	/* Clear DQS pull ups */
	writel(readl(MX6_IOM_DRAM_SDQS0) & ~0x7000, MX6_IOM_DRAM_SDQS0);
	writel(readl(MX6_IOM_DRAM_SDQS1) & ~0x7000, MX6_IOM_DRAM_SDQS1);
	writel(readl(MX6_IOM_DRAM_SDQS2) & ~0x7000, MX6_IOM_DRAM_SDQS2);
	writel(readl(MX6_IOM_DRAM_SDQS3) & ~0x7000, MX6_IOM_DRAM_SDQS3);
	writel(readl(MX6_IOM_DRAM_SDQS4) & ~0x7000, MX6_IOM_DRAM_SDQS4);
	writel(readl(MX6_IOM_DRAM_SDQS5) & ~0x7000, MX6_IOM_DRAM_SDQS5);
	writel(readl(MX6_IOM_DRAM_SDQS6) & ~0x7000, MX6_IOM_DRAM_SDQS6);
	writel(readl(MX6_IOM_DRAM_SDQS7) & ~0x7000, MX6_IOM_DRAM_SDQS7);

	/* Re-enable SDE (chip selects) if they were set initially */
	if (cs1_enable_initial == 1)
		/* Set SDE_1 */
		mmdcp0_setbit(30, MDCTL);

	if (cs0_enable_initial == 1)
		/* Set SDE_0 */
		mmdcp0_setbit(31, MDCTL);

	/* Re-enable to auto refresh */
	write_mmdcp0(temp_ref, MDREF);

	/* Clear the MDSCR (including the con_req bit) */
	write_mmdcp0(0x0, MDSCR); // CS0

	/* Poll to make sure the con_ack bit is clear */
	while ((read_mmdcp0(MDSCR) & 0x00004000))
		debug("c");

	/*
	 * Print out the registers that were updated as a result
	 * of the calibration process.
	 */
	debug("MMDC registers updated from calibration\n");
	debug("Read DQS gating calibration:\n");
	debug("\tMPDGCTRL0 PHY0 (0x021b083c) = 0x%08X\n", readl(MPDGCTRL0_PHY0));
	debug("\tMPDGCTRL1 PHY0 (0x021b0840) = 0x%08X\n", readl(MPDGCTRL1_PHY0));
	debug("\tMPDGCTRL0 PHY1 (0x021b483c) = 0x%08X\n", readl(MPDGCTRL0_PHY1));
	debug("\tMPDGCTRL1 PHY1 (0x021b4840) = 0x%08X\n", readl(MPDGCTRL1_PHY1));
	debug("Read calibration:\n");
	debug("\tMPRDDLCTL PHY0 (0x021b0848) = 0x%08X\n", readl(MPRDDLCTL_PHY0));
	debug("\tMPRDDLCTL PHY1 (0x021b4848) = 0x%08X\n", readl(MPRDDLCTL_PHY1));
	debug("Write calibration:\n");
	debug("\tMPWRDLCTL PHY0 (0x021b0850) = 0x%08X\n", readl(MPWRDLCTL_PHY0));
	debug("\tMPWRDLCTL PHY1 (0x021b4850) = 0x%08X\n", readl(MPWRDLCTL_PHY1));
	/*
	 * Registers below are for debugging purposes.  These print out
	 * the upper and lower boundaries captured during
	 * read DQS gating calibration.
	 */
	debug("Status registers bounds for read DQS gating:\n");
	debug("\tMPDGHWST0 PHY0 (0x021b087c) = 0x%08X\n", readl(0x021b087c));
	debug("\tMPDGHWST1 PHY0 (0x021b0880) = 0x%08X\n", readl(0x021b0880));
	debug("\tMPDGHWST2 PHY0 (0x021b0884) = 0x%08X\n", readl(0x021b0884));
	debug("\tMPDGHWST3 PHY0 (0x021b0888) = 0x%08X\n", readl(0x021b0888));
	debug("\tMPDGHWST0 PHY1 (0x021b487c) = 0x%08X\n", readl(0x021b487c));
	debug("\tMPDGHWST1 PHY1 (0x021b4880) = 0x%08X\n", readl(0x021b4880));
	debug("\tMPDGHWST2 PHY1 (0x021b4884) = 0x%08X\n", readl(0x021b4884));
	debug("\tMPDGHWST3 PHY1 (0x021b4888) = 0x%08X\n", readl(0x021b4888));

	debug("Final do_dqs_calibration error mask: 0x%x\n", errors);

	return errors;
}

uint32_t novena_read_spd(struct mx6_ddr_sysinfo *sysinfo,
			 struct mx6_ddr3_cfg *cfg)
{
	struct ddr3_spd_eeprom_s spd;
	int ret;
	int chip   = 0x50;	/* Address of the SPD I2C device */
	int addr   = 0x0;	/* Start of SPD address */
	int alen   = 0x1;	/* Address is one-byte long */
	uint32_t capacity;
	uint32_t mtb_scale;

	i2c_init(CONFIG_SYS_I2C_SPEED, 0);

	ret = i2c_read(chip, addr, alen, (uint8_t *)&spd, sizeof(spd));
	if (ret != 0) {
		printf("Error reading SPD on DDR3.\n");
		dram_fatal(-ret);
	}

	switch (spd.tck_min) {
	case 0x14:
		cfg->mem_speed = 800;
		if (spd.tfaw_min == 0x40) {
			cfg->pagesz = 1;
			debug("\t1K page size\n");
		} else if (spd.tfaw_min == 0x90) {
			cfg->pagesz = 2;
			debug("\t2K page size\n");
		}
		else
			debug("Unknown page size: 0x%02x\n", spd.tfaw_min);
		break;
	case 0x0f:
		cfg->mem_speed = 1066;
		if (spd.tfaw_min == 0x2c) {
			cfg->pagesz = 1;
			debug("\t1K page size\n");
		} else if (spd.tfaw_min == 0x90) {
			cfg->pagesz = 2;
			debug("\t2K page size\n");
		}
		else
			debug("Unknown page size: 0x%02x\n", spd.tfaw_min);
		break;
	case 0x0c:
		cfg->mem_speed = 1333;
		if (spd.tfaw_min == 0xf0) {
			cfg->pagesz = 1;
			debug("\t1K page size\n");
		} else if (spd.tfaw_min == 0x68) {
			cfg->pagesz = 2;
			debug("\t2K page size\n");
		}
		else
			debug("Unknown page size: 0x%02x\n", spd.tfaw_min);
		break;
	case 0x0a:
		cfg->mem_speed = 1600;
		if (spd.tfaw_min == 0xf0) {
			cfg->pagesz = 1;
			debug("\t1K page size\n");
		} else if (spd.tfaw_min == 0x40) {
			cfg->pagesz = 2;
			debug("\t2K page size\n");
		}
		else
			debug("Unknown page size: 0x%02x\n", spd.tfaw_min);
		break;
	case 0x09:
		cfg->pagesz = 1;
		cfg->mem_speed = 1866;
		break;
	case 0x08:
		cfg->pagesz = 1;
		cfg->mem_speed = 2133;
		break;
	default:
		if (spd.tck_min <= 0xf) {
			debug("**undecodable but sufficiently fast "
					"clock rate detected\n");
			cfg->mem_speed = 1066;
		} else {
			debug("**undecodable but too slow "
					"clock rate detected\n");
			cfg->mem_speed = 800;
		}
		break;
	}
	debug("\tDDR3-%d speed rating detected\n", cfg->mem_speed);
	if (cfg->mem_speed < 1066) {
		printf("memory is too slow.\n");
		dram_fatal(0x05);
	}

	cfg->density = (256 * (1 << (spd.density_banks & 0xf))) / 1024;
	debug("\tIndividual chip density is %d Gib\n", cfg->density);

	cfg->width = (1 << (spd.organization & 0x7)) * 4;
	debug("\tChips have a width of %d bits\n", cfg->width);

	if ((spd.density_banks & 0x30) != 0) {
		printf("\tWarning: memory has an unsupported bank size\n");
		dram_fatal(0x06);
	}
	cfg->banks = 8;
	debug("\t%d address banks\n", cfg->banks);

	cfg->rowaddr = ((spd.addressing >> 3) & 0x7) + 12;
	cfg->coladdr = (spd.addressing & 0x7) + 9;
	debug("\tRows: %d, Cols: %d\n", cfg->rowaddr, cfg->coladdr);

	sysinfo->ncs = ((spd.organization >> 3) & 0x7) + 1;
	debug("\tThere are %d chip selects (ranks)\n", sysinfo->ncs);

	capacity = (64 / cfg->width) * sysinfo->ncs * (cfg->density * 1024);
	debug("\tTotal memory capacity: %d Mib (%d MiB)\n",
			capacity, capacity / 8);

	/* We only support 64-bit modules */
	if (spd.bus_width != 0x3) {
		printf("Unsupported device width, fatal.\n");
		dram_fatal(0x03);
	}
	sysinfo->dsize = 2;
	debug("\tModule width is 64 bits, no ECC\n");

	sysinfo->cs_density = (capacity / (256 * sysinfo->ncs)) / 4;
	debug("\tCS density: %d Gb\n", sysinfo->cs_density);

	/*
	 * MTB is in periods of some fraction of nanoseconds.  Convert to
	 * ns*100, which is what MX6 uses.
	 */
	mtb_scale = (spd.mtb_dividend * 100) / spd.mtb_divisor;

	cfg->trcd = spd.trcd_min + 1;
	cfg->trcd *= mtb_scale;
	debug("\ttRCD: %d\n", cfg->trcd);

	cfg->trcmin = (((spd.tras_trc_ext & 0xf0) << 4) | spd.trc_min_lsb) + 1;
	cfg->trcmin *= mtb_scale;
	debug("\ttRC: %d\n", cfg->trcmin);

	cfg->trasmin = (((spd.tras_trc_ext & 0xf) << 8) | spd.tras_min_lsb) + 1;
	cfg->trasmin *= mtb_scale;
	debug("\ttRAS: %d\n", cfg->trasmin);

	/* Set self-refresh temperature flag to extended range, if present */
	cfg->SRT = 0;
	if ((spd.therm_ref_opt & 0x1) && !(spd.therm_ref_opt & 0x2))
		cfg->SRT = 1;

	/* Items with (*) will be calibrated later */
	sysinfo->cs1_mirror = 0;	/* Novena doesn't use mirroring */
	sysinfo->bi_on = 0;		/* Novena doesn't use interleaving */
	sysinfo->rtt_nom = 2;		/* (*) RTT_Nom = RZQ/2 */
	sysinfo->rtt_wr = 2;		/* (*) RTT_Wr = RZQ/4 */
	sysinfo->ralat = 3;		/* (*) Read additional latency */
	sysinfo->walat = 3;		/* (*) Write additional latency */
	sysinfo->mif3_mode = 3;		/* Command prediction working mode */
	sysinfo->rst_to_cke = 0x23;	/* 33 cycles, 500us (JEDEC default) */
	sysinfo->sde_to_rst = 0x10;	/* 14 cycles, 200us (JEDEC default) */

	return 0;
}

int novena_memory_test(void)
{
#ifdef CONFIG_PERFORM_RAM_TEST
	write_random_data(0);
	return read_random_data(0);
#endif
	return 0;
}
