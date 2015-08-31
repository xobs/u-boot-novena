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
#include <asm/arch/mx6-pins.h>
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

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

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

	/* Enable reset */
	gpio_set_value(CONFIG_PCIE_IMX_PERST_GPIO, 0);

	/* Enable the port (turn off rfkill) */
	gpio_set_value(CONFIG_PCIE_IMX_RFKILL_GPIO, 1);
#else
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);
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
	printf("PMIC: PFUZE100 ID=0x%02x\n", reg);

	/* Set SWBST to 5.0V and enable (for USB) */
	pmic_reg_read(p, PFUZE100_SWBSTCON1, &reg);
	reg &= ~(SWBST_MODE_MASK | SWBST_VOL_MASK);
	reg |= (SWBST_5_00V | SWBST_MODE_AUTO);
	pmic_reg_write(p, PFUZE100_SWBSTCON1, reg);

	return 0;
}

#if defined(CONFIG_NOVENA_LID_FEATURE)
static int power_off_if_lid_shut(void)
{
	int ret;
	uchar buffer[1];
	int tries;

	iomux_v3_cfg_t lid_switch_pads[] = {
		/* Lid Switch */
		MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	};

	imx_iomux_v3_setup_multiple_pads(lid_switch_pads,
					 ARRAY_SIZE(lid_switch_pads));

	gpio_direction_input(CONFIG_NOVENA_LID_GPIO);

	if (gpio_get_value(CONFIG_NOVENA_LID_GPIO))
		return 0;

	/* Power off, as the lid is shut */
	ret = i2c_set_bus_num(0);
	if (ret) {
		printf("Cannot select Senoko I2C bus.\n");
		return 1;
	}

	for (tries = 1; tries <= 10; tries++) {
		buffer[0] = 0x81;
		ret = i2c_write(0x20, 0x0f, 1, buffer, sizeof(buffer));

		if (ret)
			printf("Cannot power off senoko (try %d/10)\n", tries);
		mdelay(100);
	}

	return 1;
}
#endif /* CONFIG_NOVENA_LID_FEATURE */

/* EEPROM configuration data */
static struct novena_eeprom_data {
	uint8_t		signature[6];
	uint8_t		version;
	uint8_t		reserved;
	uint32_t	serial;
	uint8_t		mac[6];
	uint16_t	features;
} eeprom_data;

enum feature_flags {
	feature_es8328		= 0x0001,
	feature_senoko		= 0x0002,
	feature_retina		= 0x0004,
	feature_pixelqi		= 0x0008,
	feature_pcie		= 0x0010,
	feature_gbit		= 0x0020,
	feature_hdmi		= 0x0040,
	feature_eepromoops	= 0x0080,
	feature_rootsrc_sata	= 0x0100,
	feature_heirloom	= 0x0200,
	feature_lidbootblock	= 0x0400,
};

static int is_valid_eeprom_data(void)
{
	const char *signature = "Novena";

	/* Check EEPROM signature. */
	return !memcmp(eeprom_data.signature, signature, 6);
}

#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT)
#include <libfdt.h>
static int fdt_del_by_path(void *fdt, const char *path)
{
	int nodeoffset;	/* node offset from libfdt */
	int err;

	/*
	 * Get the path.  The root node is an oddball, the offset
	 * is zero and has no name.
	 */
	nodeoffset = fdt_path_offset(working_fdt, path);
	if (nodeoffset < 0)
		return 1;

	err = fdt_del_node(working_fdt, nodeoffset);
	if (err < 0)
		return err;

	return 0;
}

static int add_console_arg(void *fdt, char* arg)
{
	int nodeoffset = fdt_subnode_offset(fdt, 0, "chosen");
	if (nodeoffset < 0)
		return nodeoffset;

	int argerr;
	const struct fdt_property* args;
	args = fdt_get_property(fdt, nodeoffset, "bootargs", &argerr);
	if (argerr < 0)
		return argerr;

	const char* bootargs = args->data;
	const char* mid = " console=";
	char finalbootargs[CONFIG_SYS_CBSIZE];

	int newlen = strlen(bootargs) + strlen(mid) + strlen(arg);
	if (newlen > sizeof(finalbootargs)) {
		printf("finalbootarg overflow %zd+%zd+%zd+1 > %zd\n",
						strlen(bootargs), strlen(mid), strlen(arg), sizeof(finalbootargs));
		return 0;
	}

	strcpy(finalbootargs, bootargs);
	strcat(finalbootargs, mid);
	strcat(finalbootargs, arg);

	int err = fdt_setprop(fdt, nodeoffset, "bootargs", finalbootargs, newlen + 1);
	if (err < 0)
		return err;

	return 0;
}

int ft_board_setup(void *blob, bd_t *bd)
{
	if (!is_valid_eeprom_data())
		return 0;

	if (!getenv("keep_lcd"))
		fdt_del_by_path(blob, "/soc/aips-bus@02000000/ldb@020e0008");

	char* console = getenv("console-bootarg");
	if (console) {
		int err = add_console_arg(blob, console);
		if (err < 0) {
			printf("WARNING: could not add console bootarg %s.\n",
							fdt_strerror(err));
		}
	}

	if (getenv("rec")) {
		puts("Detected recovery mode, leaving everything enabled\n");
		return 0;
	}

	if (!(eeprom_data.features & feature_es8328))
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/es8328@11");

	if (!(eeprom_data.features & feature_senoko)) {
		fdt_del_by_path(blob, "soc/aips-bus@02100000/i2c@021a0000/bq20z75@0b");
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a0000/senoko@20");
	}

	if (!(eeprom_data.features & feature_pcie))
		fdt_del_by_path(blob, "/soc/pcie@0x01000000");

	if (!(eeprom_data.features & feature_gbit))
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000");

	if (!(eeprom_data.features & feature_retina)) {
		fdt_del_by_path(blob, "/soc/aips-bus@02000000/ldb@020e0008");
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/it6251@5c");
	}

	if (!(eeprom_data.features & feature_heirloom)) {
		fdt_del_by_path(blob, "/gpio-fan");
		fdt_del_by_path(blob, "/thermal-zones/cpu-thermal/cooling-maps/map0");
	}

	return 0;
}
#endif

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
 
	if (is_valid_eeprom_data() && (eeprom_data.features & feature_rootsrc_sata))
		setenv("sata_root", "yes");

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

	if (!is_valid_eeprom_data()) {
		puts("EEPROM is uninitialized\n");
		return 1;
	}

	return 0;
}

int misc_init_r(void)
{
	int ret = 0;

	if (!read_eeprom()) {
#if defined(CONFIG_NOVENA_LID_FEATURE)
		if (eeprom_data.features & feature_lidbootblock)
			power_off_if_lid_shut();
#endif /* CONFIG_NOVENA_LID_FEATURE */
	}

	ret |= set_bootdev();
	ret |= set_ethaddr();

	return ret;
}
