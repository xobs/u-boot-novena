#ifndef __NOVENA_EEPROM_H__
#define __NOVENA_EEPROM_H__

#define NOVENA_SIGNATURE "Novena"

/* Bitmask polarities used as flag */
enum modesetting_flags {

	/* If 0, this display device is not present */
	channel_present	= 0x01,

	/* If this is lvds1 and this bit is set, use dual-channel LVDS */
	dual_channel	= 0x02,

	vsync_polarity	= 0x04,
	hsync_polarity	= 0x08,

	/* For LVDS, a value of 0 means PSWG, a value of 1 means JEIDA */
	mapping_jeida	= 0x10,

	/* If 0, channel is either 6-bit (LVDS) or 10-bit (HDMI) */
	data_width_8bit	= 0x20,

	/* Channel is present, but ignore EEPROM values and auto-detect them */
	ignore_settings	= 0x40,
};

struct modesetting {
	uint32_t	frequency;
	uint16_t	hactive;
	uint16_t	vactive;
	uint16_t	hback_porch;
	uint16_t	hfront_porch;
	uint16_t	hsync_len;
	uint16_t	vback_porch;
	uint16_t	vfront_porch;
	uint16_t	vsync_len;
	uint32_t	flags;		/* enum modesetting_flags mask */
} __attribute__((__packed__));

/*
 * For structure documentation, see:
 * http://www.kosagi.com/w/index.php?title=Novena/EEPROM
 */
struct novena_eeprom_data_v2 {
	uint8_t		signature[6];	/* 'Novena' */
	uint8_t		version;	/* always 2 */
	uint8_t		page_size;	/* Size of EEPROM read/write page */
	uint32_t	serial;		/* 32-bit serial number */
	uint8_t		mac[6];		/* Gigabit MAC address */

	/* Features present, from struct feature features[] below */
	uint16_t	features;	/* Native byte order */

	/* Describes default resolutions of various output devices */
	struct modesetting	lvds1;
	struct modesetting	lvds2;
	struct modesetting	hdmi;

	/* An indicator of how large this particular EEPROM is */
	uint32_t	eeprom_size;

	/* If eepromoops is present, describes eepromoops storage */
	uint32_t	eepromoops_offset;
	uint32_t	eepromoops_length;
} __attribute__((__packed__));

/* V1 is mostly a copy of v2, except the page size is a "reserved" field */
struct novena_eeprom_data_v1 {
	uint8_t		signature[6];	/* 'Novena' */
	uint8_t		version;	/* always 1 */
	uint8_t		reserved;
	uint32_t	serial;		/* 32-bit serial number */
	uint8_t		mac[6];		/* Gigabit MAC address */

	/* Features present, from struct feature features[] below */
	uint16_t	features;	/* Native byte order */
} __attribute__((__packed__));

enum feature_flags {
	feature_es8328 		= 0x0001,
	feature_senoko		= 0x0002,
	feature_retina		= 0x0004,
	feature_pixelqi		= 0x0008,
	feature_pcie		= 0x0010,
	feature_gbit		= 0x0020,
	feature_hdmi		= 0x0040,
	feature_eepromoops	= 0x0080,
	feature_rootsrc_sata	= 0x0100,
};

#ifndef __cplusplus
struct feature {
	uint32_t	flags;
	char		*name;
	char		*descr;
} features[] = {
	{
		.name	= "es8328",
		.flags	= feature_es8328,
		.descr	= "ES8328 audio codec",
	},
	{
		.name	= "senoko",
		.flags	= feature_senoko,
		.descr	= "Senoko battery board",
	},
	{
		.name	= "retina",
		.flags	= feature_retina,
		.descr	= "Retina-class dual-LVDS display (deprecated)",
	},
	{
		.name	= "pixelqi",
		.flags	= feature_pixelqi,
		.descr	= "PixelQi LVDS display (deprecated)",
	},
	{
		.name	= "pcie",
		.flags	= feature_pcie,
		.descr	= "PCI Express support",
	},
	{
		.name	= "gbit",
		.flags	= feature_gbit,
		.descr	= "Gigabit Ethernet",
	},
	{
		.name	= "hdmi",
		.flags	= feature_hdmi,
		.descr	= "HDMI Output (deprecated)",
	},
	{
		.name	= "eepromoops",
		.flags	= feature_eepromoops,
		.descr	= "EEPROM Oops storage",
	},
	{} /* Sentinal */
};
#endif /* ! __cplusplus */

#endif /* __NOVENA_EEPROM_H__ */
