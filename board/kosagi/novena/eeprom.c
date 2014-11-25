#include <common.h>
#include <i2c.h>

#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT)
#include <libfdt.h>
#include "novena-eeprom.h"

#define EEPROM_ADDRESS (0xac >> 1)

static int fdt_del_by_path(void *fdt, const char *path)
{
	int  nodeoffset;        /* node offset from libfdt */
	int  err;

	/*
	 * Get the path.  The root node is an oddball, the offset
	 * is zero and has no name.
	 */
	nodeoffset = fdt_path_offset(working_fdt, path);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		printf ("libfdt fdt_path_offset() returned %s\n",
			fdt_strerror(nodeoffset));
		return 1;
	}

	err = fdt_del_node(working_fdt, nodeoffset);
	if (err < 0) {
		printf("libfdt fdt_del_by_path():  %s\n",
		fdt_strerror(err));
		return err;
	}

	return 0;
}

void ft_board_setup(void *blob, bd_t *bd)
{
	struct novena_eeprom_data_v2 data;
	int ret;

	i2c_set_bus_num(2);
	ret = eeprom_read(EEPROM_ADDRESS, 0, (uint8_t *)&data, sizeof(data));
	if (ret != 0) {
		printf("Unable to read from EEPROM: %d\n", ret);
		return;
	}

	/* Check EEPROM signature. */
	if (memcmp(data.signature, NOVENA_SIGNATURE, sizeof(data.signature))) {
		puts("Invalid I2C EEPROM signature.\n");
		return;
	}

	/* Check which version */
	if (data.version > 2) {
		printf("Unsupported EEPROM version: %d\n", data.version);
		return;
	}

	/* Set ethernet address from EEPROM (if one hasn't been set yet). */
	if (!getenv("ethaddr"))
		eth_setenv_enetaddr("ethaddr", data.mac);

	if (getenv("rec")) {
		puts("Detected recovery mode, leaving everything enabled\n");
		return;
	}

	if (!(data.features & feature_es8328))
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/es8328@11");
	if (!(data.features & feature_senoko))
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a0000/bq20z75@0b");
	if (!(data.features & feature_pcie))
		fdt_del_by_path(blob, "/soc/pcie@0x01000000");

	if (!(data.features & feature_gbit))
		fdt_del_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000");

	/* Older version had simpler panel selection */
	if (data.version == 1) {
		if (!(data.features & feature_hdmi))
			fdt_del_by_path(blob, "/soc/aips-bus@02000000/hdmi@0120000");

		if (!(data.features & feature_retina)) {
			fdt_del_by_path(blob, "/soc/aips-bus@02000000/ldb@020e0008");
			fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/it6251@5c");
		}
	}

	else if (data.version == 2) {
		/* In version 2, this indicates whether an LVDS-to-eDP chip is present */
		if (!(data.features & feature_retina))
			fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/it6251@5c");

		if (!(data.lvds1.flags & channel_present)) {
			fdt_del_by_path(blob, "/soc/aips-bus@02000000/ldb@020e0008/lvds-channel@0");
		}
		else {
			/* XXX Configure lvds1 display timings here */
		}

		if (!(data.lvds2.flags & channel_present) &&
		    !(data.lvds2.flags & dual_channel)) {
			fdt_del_by_path(blob, "/soc/aips-bus@02000000/ldb@020e0008/lvds-channel@1");
		}
		else {
			/* XXX Configure lvds2 display timings here */
		}

		if (!(data.hdmi.flags & channel_present)) {
			fdt_del_by_path(blob, "/soc/aips-bus@02000000/hdmi@0120000");
		}
		else if (data.hdmi.flags & ignore_settings) {
			/* Do nothing, let the system auto-detect */
			;
		}
		else {
			/* XXX Configure hdmi display timings here */
		}

		if (!(data.features & feature_eepromoops))
			fdt_del_by_path(blob, "/soc/aips-bus@02100000/i2c@021a8000/eepromoops@56");
		else {
			/* XXX Configure eepromoops here */
		}

		if (data.features & feature_rootsrc_sata) {
			setenv("rootdev", "PARTUUID=4e6f7653-03"); /* NovS */ \
			char tmpargs[1024];
			strcpy(tmpargs, getenv("bootargs"));
			strcat(tmpargs, " resume=PARTUUID=4e6f7653-02"); /* NovS */ \
			setenv("bootargs", tmpargs);
		}
	}
}

#endif
