/*
 * Novena board support
 *
 * Copyright (C) 2014 Marek Vasut <marex@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/video.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <input.h>
#include <ipu_pixfmt.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <stdio_dev.h>

#include "novena.h"

DECLARE_GLOBAL_DATA_PTR;

enum boot_device {
	MX6_SD0_BOOT,
	MX6_SD1_BOOT,
	MX6_MMC_BOOT,
	MX6_NAND_BOOT,
	MX6_SATA_BOOT,
	MX6_WEIM_NOR_BOOT,
	MX6_ONE_NAND_BOOT,
	MX6_PATA_BOOT,
	MX6_I2C_BOOT,
	MX6_SPI_NOR_BOOT,
	MX6_UNKNOWN_BOOT,
	MX6_BOOT_DEV_NUM = MX6_UNKNOWN_BOOT,
};

static enum boot_device get_boot_device(void)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned soc_sbmr = readl(&psrc->sbmr1);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;
	uint bt_mem_mmc = (soc_sbmr & 0x00001000) >> 12;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			return MX6_ONE_NAND_BOOT;
		else
			return MX6_WEIM_NOR_BOOT;
		break;
	case 0x2:
			return MX6_SATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			return MX6_I2C_BOOT;
		else
			return MX6_SPI_NOR_BOOT;
		break;
	case 0x4:
	case 0x5:
		if (bt_mem_mmc)
			return MX6_SD0_BOOT;
		else
			return MX6_SD1_BOOT;
		break;
	case 0x6:
	case 0x7:
		return MX6_MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		return MX6_NAND_BOOT;
		break;
	default:
		return MX6_UNKNOWN_BOOT;
		break;
	}
}

/*
 * GPIO button
 */
#ifdef CONFIG_KEYBOARD
static struct input_config button_input;

static int novena_gpio_button_read_keys(struct input_config *input)
{
	int key = KEY_ENTER;
	if (gpio_get_value(NOVENA_BUTTON_GPIO))
		return 0;
	input_send_keycodes(&button_input, &key, 1);
	return 1;
}

static int novena_gpio_button_getc(struct stdio_dev *dev)
{
	return input_getc(&button_input);
}

static int novena_gpio_button_tstc(struct stdio_dev *dev)
{
	return input_tstc(&button_input);
}

static int novena_gpio_button_init(struct stdio_dev *dev)
{
	gpio_direction_input(NOVENA_BUTTON_GPIO);
	input_set_delays(&button_input, 250, 250);
	return 0;
}

int drv_keyboard_init(void)
{
	int error;
	struct stdio_dev dev = {
		.name	= "button",
		.flags	= DEV_FLAGS_INPUT | DEV_FLAGS_SYSTEM,
		.start	= novena_gpio_button_init,
		.getc	= novena_gpio_button_getc,
		.tstc	= novena_gpio_button_tstc,
	};

	error = input_init(&button_input, 0);
	if (error) {
		debug("%s: Cannot set up input\n", __func__);
		return -1;
	}
	button_input.read_keys = novena_gpio_button_read_keys;

	error = input_stdio_register(&dev);
	if (error)
		return error;

	return 0;
}
#endif

/*
 * SDHC
 */
#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{ USDHC3_BASE_ADDR, 0, 4 },	/* Micro SD */
	{ USDHC2_BASE_ADDR, 0, 4 },	/* Big SD */
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	/* There is no CD for a microSD card, assume always present. */
	if (cfg->esdhc_base == USDHC3_BASE_ADDR)
		return 1;
	else
		return !gpio_get_value(NOVENA_SD_CD);
}

int board_mmc_getwp(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	/* There is no WP for a microSD card, assume always read-write. */
	if (cfg->esdhc_base == USDHC3_BASE_ADDR)
		return 0;
	else
		return gpio_get_value(NOVENA_SD_WP);
}


int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int index;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	/* Big SD write-protect and card-detect */
	gpio_direction_input(NOVENA_SD_WP);
	gpio_direction_input(NOVENA_SD_CD);

	for (index = 0; index < ARRAY_SIZE(usdhc_cfg); index++) {
		status = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (status)
			return status;
	}

	return status;
}
#endif

int board_early_init_f(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display_clock();
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

#ifdef CONFIG_PCI
	/* Power on the PCIe port to prevent kernel from locking up */
	gpio_set_value(CONFIG_PCIE_IMX_POWER_GPIO, 1);
#endif

	return 0;
}

int board_late_init(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display_lvds();
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: Novena 4x\n");
	return 0;
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

/* setup board specific PMIC */
int power_init_board(void)
{
	struct pmic *p;
	u32 reg;
	int ret;

	power_pfuze100_init(1);
	p = pmic_get("PFUZE100");
	if (!p)
		return -EINVAL;

	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
	printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);

	/* Set SWBST to 5.0V and enable (for USB) */
	pmic_reg_read(p, PFUZE100_SWBSTCON1, &reg);
	reg &= ~(SWBST_MODE_MASK | SWBST_VOL_MASK);
	reg |= (SWBST_5_00V | SWBST_MODE_AUTO);
	pmic_reg_write(p, PFUZE100_SWBSTCON1, reg);

	return 0;
}

/* EEPROM configuration data */
static struct novena_eeprom_data {
	uint8_t		signature[6];
	uint8_t		version;
	uint8_t		reserved;
	uint32_t	serial;
	uint8_t		mac[6];
	uint16_t	features;
} eeprom_data;

static int is_valid_eeprom_data(void)
{
	const char *signature = "Novena";

	/* Check EEPROM signature. */
	return !memcmp(eeprom_data.signature, signature, 6);
}

static int set_bootdev(void)
{
	/* Set boot environment variables */
	switch(get_boot_device()) {
	case MX6_SD0_BOOT:
		setenv("bootdev", "0");
		setenv("bootsrc", "mmc");
		break;

	case MX6_SD1_BOOT:
		setenv("bootdev", "1");
		setenv("bootsrc", "mmc");
		break;

	case MX6_SATA_BOOT:
		setenv("bootdev", "0");
		setenv("bootsrc", "sata");
		break;

	default:
		return 1;
		break;
 	}
 
	if (is_valid_eeprom_data() && (eeprom_data.features & 0x100))
		setenv("rootdev", "PARTUUID=4e6f7653-03"); /* NovS */
	else
		setenv("rootdev", "PARTUUID=4e6f764d-03"); /* NovM */

	return 0;
}

static int set_ethaddr(void)
{
	/* If 'ethaddr' is already set, do nothing. */
	if (getenv("ethaddr"))
		return 0;

	/* Set ethernet address from EEPROM. */
	if (is_valid_eeprom_data())
		eth_setenv_enetaddr("ethaddr", eeprom_data.mac);

	return 0;
}

static int read_eeprom(void)
{
	int ret;

	/* EEPROM is at bus 2. */
	ret = i2c_set_bus_num(2);
	if (ret) {
		puts("Cannot select EEPROM I2C bus.\n");
		return 1;
	}

	/* EEPROM is at address 0x56. */
	ret = eeprom_read(0x56, 0, (void *)&eeprom_data, sizeof(eeprom_data));
	if (ret) {
		puts("Cannot read I2C EEPROM.\n");
		return ret;
	}

	if (!is_valid_eeprom_data())
		puts("EEPROM is uninitialized\n");
	return 0;
}

int misc_init_r(void)
{
	int ret = 0;

	read_eeprom();
	ret |= set_bootdev();
	ret |= set_ethaddr();

	return ret;
}
