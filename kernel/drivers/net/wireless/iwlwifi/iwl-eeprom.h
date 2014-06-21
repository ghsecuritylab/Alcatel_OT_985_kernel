

#ifndef __iwl_eeprom_h__
#define __iwl_eeprom_h__

#include <net/mac80211.h>

struct iwl_priv;

#define IWL_EEPROM_ACCESS_TIMEOUT	5000 /* uSec */

#define IWL_EEPROM_SEM_TIMEOUT 		10   /* microseconds */
#define IWL_EEPROM_SEM_RETRY_LIMIT	1000 /* number of attempts (not time) */


#define IWL_NUM_TX_CALIB_GROUPS 5
enum {
	EEPROM_CHANNEL_VALID = (1 << 0),	/* usable for this SKU/geo */
	EEPROM_CHANNEL_IBSS = (1 << 1),		/* usable as an IBSS channel */
	/* Bit 2 Reserved */
	EEPROM_CHANNEL_ACTIVE = (1 << 3),	/* active scanning allowed */
	EEPROM_CHANNEL_RADAR = (1 << 4),	/* radar detection required */
	EEPROM_CHANNEL_WIDE = (1 << 5),		/* 20 MHz channel okay */
	/* Bit 6 Reserved (was Narrow Channel) */
	EEPROM_CHANNEL_DFS = (1 << 7),	/* dynamic freq selection candidate */
};

/* SKU Capabilities */
#define EEPROM_SKU_CAP_SW_RF_KILL_ENABLE                (1 << 0)
#define EEPROM_SKU_CAP_HW_RF_KILL_ENABLE                (1 << 1)

struct iwl_eeprom_channel {
	u8 flags;		/* EEPROM_CHANNEL_* flags copied from EEPROM */
	s8 max_power_avg;	/* max power (dBm) on this chnl, limit 31 */
} __attribute__ ((packed));

struct iwl_eeprom_enhanced_txpwr {
	__le16 common;
	s8 chain_a_max;
	s8 chain_b_max;
	s8 chain_c_max;
	s8 reserved;
	s8 mimo2_max;
	s8 mimo3_max;
} __attribute__ ((packed));

/* 3945 Specific */
#define EEPROM_3945_EEPROM_VERSION	(0x2f)

/* 4965 has two radio transmitters (and 3 radio receivers) */
#define EEPROM_TX_POWER_TX_CHAINS      (2)

/* 4965 has room for up to 8 sets of txpower calibration data */
#define EEPROM_TX_POWER_BANDS          (8)

#define EEPROM_TX_POWER_MEASUREMENTS   (3)

/* 4965 Specific */
/* 4965 driver does not work with txpower calibration version < 5 */
#define EEPROM_4965_TX_POWER_VERSION    (5)
#define EEPROM_4965_EEPROM_VERSION	(0x2f)
#define EEPROM_4965_CALIB_VERSION_OFFSET       (2*0xB6) /* 2 bytes */
#define EEPROM_4965_CALIB_TXPOWER_OFFSET       (2*0xE8) /* 48  bytes */
#define EEPROM_4965_BOARD_REVISION             (2*0x4F) /* 2 bytes */
#define EEPROM_4965_BOARD_PBA                  (2*0x56+1) /* 9 bytes */

/* 5000 Specific */
#define EEPROM_5000_TX_POWER_VERSION    (4)
#define EEPROM_5000_EEPROM_VERSION	(0x11A)

/* 5000 and up calibration */
#define EEPROM_CALIB_ALL	(INDIRECT_ADDRESS | INDIRECT_CALIBRATION)
#define EEPROM_XTAL		((2*0x128) | EEPROM_CALIB_ALL)

/* 5000 temperature */
#define EEPROM_5000_TEMPERATURE ((2*0x12A) | EEPROM_CALIB_ALL)

/* agn links */
#define EEPROM_LINK_HOST             (2*0x64)
#define EEPROM_LINK_GENERAL          (2*0x65)
#define EEPROM_LINK_REGULATORY       (2*0x66)
#define EEPROM_LINK_CALIBRATION      (2*0x67)
#define EEPROM_LINK_PROCESS_ADJST    (2*0x68)
#define EEPROM_LINK_OTHERS           (2*0x69)

/* agn regulatory - indirect access */
#define EEPROM_REG_BAND_1_CHANNELS       ((0x08)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 28 bytes */
#define EEPROM_REG_BAND_2_CHANNELS       ((0x26)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 26 bytes */
#define EEPROM_REG_BAND_3_CHANNELS       ((0x42)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 24 bytes */
#define EEPROM_REG_BAND_4_CHANNELS       ((0x5C)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 22 bytes */
#define EEPROM_REG_BAND_5_CHANNELS       ((0x74)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 12 bytes */
#define EEPROM_REG_BAND_24_HT40_CHANNELS  ((0x82)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 14  bytes */
#define EEPROM_REG_BAND_52_HT40_CHANNELS  ((0x92)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 22  bytes */

/* 6000 regulatory - indirect access */
#define EEPROM_6000_REG_BAND_24_HT40_CHANNELS  ((0x80)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 14  bytes */

/* 6000 and up regulatory tx power - indirect access */
/* max. elements per section */
#define EEPROM_MAX_TXPOWER_SECTION_ELEMENTS	(8)
#define EEPROM_TXPOWER_COMMON_HT40_INDEX	(2)

/* 2.4 GHz band: CCK */
#define EEPROM_LB_CCK_20_COMMON       ((0xA8)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 8 bytes */
/* 2.4 GHz band: 20MHz-Legacy, 20MHz-HT, 40MHz-HT */
#define EEPROM_LB_OFDM_COMMON       ((0xB0)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 24 bytes */
/* 5.2 GHz band: 20MHz-Legacy, 20MHz-HT, 40MHz-HT */
#define EEPROM_HB_OFDM_COMMON       ((0xC8)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 24 bytes */
#define EEPROM_LB_OFDM_20_BAND       ((0xE0)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 64 bytes */
/* 2.4 GHz band HT40 channels: (1,+1) (2,+1) (6,+1) (7,+1) (9,+1) */
#define EEPROM_LB_OFDM_HT40_BAND       ((0x120)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 40 bytes */
/* 5.2GHz band channels: 36Legacy, 36HT, 64Legacy, 64HT, 100Legacy, 100HT */
#define EEPROM_HB_OFDM_20_BAND       ((0x148)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 48 bytes */
/* 5.2 GHz band HT40 channels: (36,+1) (60,+1) (100,+1) */
#define EEPROM_HB_OFDM_HT40_BAND       ((0x178)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 24 bytes */
/* 2.4 GHz band, channnel 13: Legacy, HT */
#define EEPROM_LB_OFDM_20_CHANNEL_13       ((0x190)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 16 bytes */
/* 5.2 GHz band, channnel 140: Legacy, HT */
#define EEPROM_HB_OFDM_20_CHANNEL_140       ((0x1A0)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 16 bytes */
/* 5.2 GHz band, HT40 channnels (132,+1) (44,+1) */
#define EEPROM_HB_OFDM_HT40_BAND_1       ((0x1B0)\
		| INDIRECT_ADDRESS | INDIRECT_REGULATORY)   /* 16 bytes */


/* 5050 Specific */
#define EEPROM_5050_TX_POWER_VERSION    (4)
#define EEPROM_5050_EEPROM_VERSION	(0x21E)

/* 1000 Specific */
#define EEPROM_1000_TX_POWER_VERSION    (4)
#define EEPROM_1000_EEPROM_VERSION	(0x15C)

/* 6x00 Specific */
#define EEPROM_6000_TX_POWER_VERSION    (4)
#define EEPROM_6000_EEPROM_VERSION	(0x434)

/* 6x50 Specific */
#define EEPROM_6050_TX_POWER_VERSION    (4)
#define EEPROM_6050_EEPROM_VERSION	(0x532)

/* 6x00g2 Specific */
#define EEPROM_6000G2_TX_POWER_VERSION    (6)
#define EEPROM_6000G2_EEPROM_VERSION	(0x709)

/* OTP */
/* lower blocks contain EEPROM image and calibration data */
#define OTP_LOW_IMAGE_SIZE		(2 * 512 * sizeof(u16)) /* 2 KB */
/* high blocks contain PAPD data */
#define OTP_HIGH_IMAGE_SIZE_6x00        (6 * 512 * sizeof(u16)) /* 6 KB */
#define OTP_HIGH_IMAGE_SIZE_1000        (0x200 * sizeof(u16)) /* 1024 bytes */
#define OTP_MAX_LL_ITEMS_1000		(3)	/* OTP blocks for 1000 */
#define OTP_MAX_LL_ITEMS_6x00		(4)	/* OTP blocks for 6x00 */
#define OTP_MAX_LL_ITEMS_6x50		(7)	/* OTP blocks for 6x50 */

/* 2.4 GHz */
extern const u8 iwl_eeprom_band_1[14];

struct iwl_eeprom_calib_measure {
	u8 temperature;		/* Device temperature (Celsius) */
	u8 gain_idx;		/* Index into gain table */
	u8 actual_pow;		/* Measured RF output power, half-dBm */
	s8 pa_det;		/* Power amp detector level (not used) */
} __attribute__ ((packed));


struct iwl_eeprom_calib_ch_info {
	u8 ch_num;
	struct iwl_eeprom_calib_measure
		measurements[EEPROM_TX_POWER_TX_CHAINS]
			[EEPROM_TX_POWER_MEASUREMENTS];
} __attribute__ ((packed));

struct iwl_eeprom_calib_subband_info {
	u8 ch_from;	/* channel number of lowest channel in subband */
	u8 ch_to;	/* channel number of highest channel in subband */
	struct iwl_eeprom_calib_ch_info ch1;
	struct iwl_eeprom_calib_ch_info ch2;
} __attribute__ ((packed));


struct iwl_eeprom_calib_info {
	u8 saturation_power24;	/* half-dBm (e.g. "34" = 17 dBm) */
	u8 saturation_power52;	/* half-dBm */
	__le16 voltage;		/* signed */
	struct iwl_eeprom_calib_subband_info
		band_info[EEPROM_TX_POWER_BANDS];
} __attribute__ ((packed));


#define ADDRESS_MSK                 0x0000FFFF
#define INDIRECT_TYPE_MSK           0x000F0000
#define INDIRECT_HOST               0x00010000
#define INDIRECT_GENERAL            0x00020000
#define INDIRECT_REGULATORY         0x00030000
#define INDIRECT_CALIBRATION        0x00040000
#define INDIRECT_PROCESS_ADJST      0x00050000
#define INDIRECT_OTHERS             0x00060000
#define INDIRECT_ADDRESS            0x00100000

/* General */
#define EEPROM_DEVICE_ID                    (2*0x08)	/* 2 bytes */
#define EEPROM_MAC_ADDRESS                  (2*0x15)	/* 6  bytes */
#define EEPROM_BOARD_REVISION               (2*0x35)	/* 2  bytes */
#define EEPROM_BOARD_PBA_NUMBER             (2*0x3B+1)	/* 9  bytes */
#define EEPROM_VERSION                      (2*0x44)	/* 2  bytes */
#define EEPROM_SKU_CAP                      (2*0x45)	/* 1  bytes */
#define EEPROM_OEM_MODE                     (2*0x46)	/* 2  bytes */
#define EEPROM_WOWLAN_MODE                  (2*0x47)	/* 2  bytes */
#define EEPROM_RADIO_CONFIG                 (2*0x48)	/* 2  bytes */
#define EEPROM_3945_M_VERSION               (2*0x4A)	/* 1  bytes */

/* The following masks are to be applied on EEPROM_RADIO_CONFIG */
#define EEPROM_RF_CFG_TYPE_MSK(x)   (x & 0x3)         /* bits 0-1   */
#define EEPROM_RF_CFG_STEP_MSK(x)   ((x >> 2)  & 0x3) /* bits 2-3   */
#define EEPROM_RF_CFG_DASH_MSK(x)   ((x >> 4)  & 0x3) /* bits 4-5   */
#define EEPROM_RF_CFG_PNUM_MSK(x)   ((x >> 6)  & 0x3) /* bits 6-7   */
#define EEPROM_RF_CFG_TX_ANT_MSK(x) ((x >> 8)  & 0xF) /* bits 8-11  */
#define EEPROM_RF_CFG_RX_ANT_MSK(x) ((x >> 12) & 0xF) /* bits 12-15 */

#define EEPROM_3945_RF_CFG_TYPE_MAX  0x0
#define EEPROM_4965_RF_CFG_TYPE_MAX  0x1

/* Radio Config for 5000 and up */
#define EEPROM_RF_CONFIG_TYPE_R3x3	0x0
#define EEPROM_RF_CONFIG_TYPE_R2x2	0x1
#define EEPROM_RF_CONFIG_TYPE_R1x2	0x2
#define EEPROM_RF_CONFIG_TYPE_MAX	0x3

#define EEPROM_REGULATORY_SKU_ID            (2*0x60)    /* 4  bytes */
#define EEPROM_REGULATORY_BAND_1            (2*0x62)	/* 2  bytes */
#define EEPROM_REGULATORY_BAND_1_CHANNELS   (2*0x63)	/* 28 bytes */

#define EEPROM_REGULATORY_BAND_2            (2*0x71)	/* 2  bytes */
#define EEPROM_REGULATORY_BAND_2_CHANNELS   (2*0x72)	/* 26 bytes */

#define EEPROM_REGULATORY_BAND_3            (2*0x7F)	/* 2  bytes */
#define EEPROM_REGULATORY_BAND_3_CHANNELS   (2*0x80)	/* 24 bytes */

#define EEPROM_REGULATORY_BAND_4            (2*0x8C)	/* 2  bytes */
#define EEPROM_REGULATORY_BAND_4_CHANNELS   (2*0x8D)	/* 22 bytes */

#define EEPROM_REGULATORY_BAND_5            (2*0x98)	/* 2  bytes */
#define EEPROM_REGULATORY_BAND_5_CHANNELS   (2*0x99)	/* 12 bytes */

#define EEPROM_4965_REGULATORY_BAND_24_HT40_CHANNELS (2*0xA0)	/* 14 bytes */

#define EEPROM_4965_REGULATORY_BAND_52_HT40_CHANNELS (2*0xA8)	/* 22 bytes */

#define EEPROM_REGULATORY_BAND_NO_HT40			(0)

struct iwl_eeprom_ops {
	const u32 regulatory_bands[7];
	int (*verify_signature) (struct iwl_priv *priv);
	int (*acquire_semaphore) (struct iwl_priv *priv);
	void (*release_semaphore) (struct iwl_priv *priv);
	u16 (*calib_version) (struct iwl_priv *priv);
	const u8* (*query_addr) (const struct iwl_priv *priv, size_t offset);
	void (*update_enhanced_txpower) (struct iwl_priv *priv);
};


void iwl_eeprom_get_mac(const struct iwl_priv *priv, u8 *mac);
int iwl_eeprom_init(struct iwl_priv *priv);
void iwl_eeprom_free(struct iwl_priv *priv);
int  iwl_eeprom_check_version(struct iwl_priv *priv);
const u8 *iwl_eeprom_query_addr(const struct iwl_priv *priv, size_t offset);
u16 iwl_eeprom_query16(const struct iwl_priv *priv, size_t offset);

int iwlcore_eeprom_verify_signature(struct iwl_priv *priv);
int iwlcore_eeprom_acquire_semaphore(struct iwl_priv *priv);
void iwlcore_eeprom_release_semaphore(struct iwl_priv *priv);
const u8 *iwlcore_eeprom_query_addr(const struct iwl_priv *priv, size_t offset);
void iwlcore_eeprom_enhanced_txpower(struct iwl_priv *priv);
int iwl_init_channel_map(struct iwl_priv *priv);
void iwl_free_channel_map(struct iwl_priv *priv);
const struct iwl_channel_info *iwl_get_channel_info(
		const struct iwl_priv *priv,
		enum ieee80211_band band, u16 channel);

#endif  /* __iwl_eeprom_h__ */
