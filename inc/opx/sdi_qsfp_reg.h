/*
 * Copyright (c) 2016 Dell Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * THIS CODE IS PROVIDED ON AN  *AS IS* BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 *  LIMITATION ANY IMPLIED WARRANTIES OR CONDITIONS OF TITLE, FITNESS
 * FOR A PARTICULAR PURPOSE, MERCHANTABLITY OR NON-INFRINGEMENT.
 *
 * See the Apache Version 2.0 License for specific language governing
 * permissions and limitations under the License.
 */

/*
 * filename: sdi_qsfp_reg.h
 */


/*******************************************************************
* @file   sdi_qsfp_reg.h
* @brief  Defines qsfp register offsets and flag values
*******************************************************************/

#ifndef __SDI_QSFP_REG_H_
#define __SDI_QSFP_REG_H_

#define IDENTIFIER_OFFSET  (0)
/**
 * QSFP Register definitions. Spec for QSFP is available
 * http://force10.dell.com/npf/Platform%20Dependent%20Hardware/eleceng/compeng/Optics%20Documents/Standards/SFF/SFF-8436.PDF
 */
typedef enum {
    QSFP_STATUS_INDICATOR_OFFSET    = 2,
    QSFP_CHANNEL_LOS_INDICATOR      = 3,
    QSFP_CHANNEL_TXFAULT_INDICATOR  = 4,
    QSFP_TEMP_INTERRUPT_OFFSET      = 6,
    QSFP_VOLT_INTERRUPT_OFFSET      = 7,
    QSFP_RX12_POWER_INTERRUPT_OFFSET= 9,
    QSFP_RX34_POWER_INTERRUPT_OFFSET= 10,
    QSFP_TX12_BIAS_INTERRUPT_OFFSET = 11,
    QSFP_TX34_BIAS_INTERRUPT_OFFSET = 12,
    QSFP_TEMPERATURE_OFFSET         = 22,
    QSFP_VOLTAGE_OFFSET             = 26,
    QSFP_RX1_POWER_OFFSET           = 34,
    QSFP_RX2_POWER_OFFSET           = 36,
    QSFP_RX3_POWER_OFFSET           = 38,
    QSFP_RX4_POWER_OFFSET           = 40,
    QSFP_TX1_POWER_BIAS_OFFSET      = 42,
    QSFP_TX2_POWER_BIAS_OFFSET      = 44,
    QSFP_TX3_POWER_BIAS_OFFSET      = 46,
    QSFP_TX4_POWER_BIAS_OFFSET      = 48,
    QSFP_TX_CONTROL_OFFSET          = 86,
    QSFP_CDR_CONTROL_OFFSET         = 98,
    QSFP_FREE_SIDE_DEV_PROP_OFFSET  = 113,
    QSFP_PAGE_SELECT_BYTE_OFFSET    = 127,
    QSFP_IDENTIFIER_OFFSET          = 128,
    QSFP_EXT_IDENTIFIER_OFFSET      = 129,
    QSFP_CONNECTOR_OFFSET           = 130,
    QSFP_COMPLIANCE_CODE_OFFSET     = 131,
    QSFP_ENCODING_TYPE_OFFSET       = 139,
    QSFP_NM_BITRATE_OFFSET          = 140,
    QSFP_LENGTH_SMF_KM_OFFSET       = 142,
    QSFP_LENGTH_OM3_OFFSET          = 143,
    QSFP_LENGTH_OM2_OFFSET          = 144,
    QSFP_LENGTH_OM1_OFFSET          = 145,
    QSFP_LENGTH_CABLE_ASSEMBLY_OFFSET = 146,
    QSFP_DEVICE_TECH_OFFSET         = 147,
    QSFP_VENDOR_NAME_OFFSET         = 148,
    QSFP_VENDOR_OUI_OFFSET          = 165,
    QSFP_VENDOR_PN_OFFSET           = 168,
    QSFP_VENDOR_REVISION_OFFSET     = 184,
    QSFP_WAVELENGTH_OFFSET          = 186,
    QSFP_WAVELENGTH_TOLERANCE_OFFSET= 188,
    QSFP_MAX_CASE_TEMP_OFFSET       = 190,
    QSFP_CC_BASE_OFFSET             = 191,
    QSFP_OPTIONS1_OFFSET            = 192,
    QSFP_OPTIONS2_OFFSET            = 193,
    QSFP_OPTIONS3_OFFSET            = 194,
    QSFP_OPTIONS4_OFFSET            = 195,
    QSFP_VENDOR_SN_OFFSET           = 196,
    QSFP_VENDOR_DATE_OFFSET         = 212,
    QSFP_DIAG_MON_TYPE_OFFSET       = 220,
    QSFP_ENHANCED_OPTIONS_OFFSET    = 221,
    QSFP_CC_EXT_OFFSET              = 223
} qsfp_reg_offset_t;

/* Table 46 — Module and Channel Thresholds (Page 03) */
typedef enum {
    QSFP_TEMP_HIGH_ALARM_THRESHOLD_OFFSET         = 128,
    QSFP_TEMP_LOW_ALARM_THRESHOLD_OFFSET          = 130,
    QSFP_TEMP_HIGH_WARNING_THRESHOLD_OFFSET       = 132,
    QSFP_TEMP_LOW_WARNING_THRESHOLD_OFFSET        = 134,
    QSFP_VOLT_HIGH_ALARM_THRESHOLD_OFFSET         = 144,
    QSFP_VOLT_LOW_ALARM_THRESHOLD_OFFSET          = 146,
    QSFP_VOLT_HIGH_WARNING_THRESHOLD_OFFSET       = 148,
    QSFP_VOLT_LOW_WARNING_THRESHOLD_OFFSET        = 150,
    QSFP_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET       = 176,
    QSFP_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET        = 178,
    QSFP_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET     = 180,
    QSFP_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET      = 182,
    QSFP_TX_BIAS_HIGH_ALARM_THRESHOLD_OFFSET      = 184,
    QSFP_TX_BIAS_LOW_ALARM_THRESHOLD_OFFSET       = 186,
    QSFP_TX_BIAS_HIGH_WARNING_THRESHOLD_OFFSET    = 188,
    QSFP_TX_BIAS_LOW_WARNING_THRESHOLD_OFFSET     = 190,
} qsfp_page3_reg_offset_t;

/* QSFP-DD register offsets */
typedef enum {
    QSFP_DD_CHANNEL_TX_LOS_INDICATOR             = 3,
    QSFP_DD_CHANNEL_RX_LOS_INDICATOR             = 4,
    QSFP_DD_CHANNEL_TX_CDR_LOL_INDICATOR         = 5,
    QSFP_DD_CHANNEL_RX_CDR_LOL_INDICATOR         = 6,
    QSFP_DD_CHANNEL_TXFAULT_INDICATOR            = 7,
    QSFP_DD_TEMP_INTERRUPT_OFFSET                = 8,
    QSFP_DD_VOLT_INTERRUPT_OFFSET                = 9,
    QSFP_DD_RX_POWER_LW_INTERRUPT_OFFSET         = 10,
    QSFP_DD_RX_POWER_LA_INTERRUPT_OFFSET         = 11,
    QSFP_DD_RX_POWER_HA_INTERRUPT_OFFSET         = 12,
    QSFP_DD_TX_BIAS_LA_INTERRUPT_OFFSET          = 13,
    QSFP_DD_TX_BIAS_HA_INTERRUPT_OFFSET          = 14,
    QSFP_DD_TX_POWER_LA_INTERRUPT_OFFSET         = 15,
    QSFP_DD_TX_POWER_HA_INTERRUPT_OFFSET         = 16,
    QSFP_DD_TEMPERATURE_OFFSET                   = 22,
    QSFP_DD_TEMPERATURE2_OFFSET                  = 24,
    QSFP_DD_VOLTAGE_OFFSET                       = 26,
    QSFP_DD_RX1_POWER_OFFSET                     = 34,
    QSFP_DD_RX2_POWER_OFFSET                     = 36,
    QSFP_DD_RX3_POWER_OFFSET                     = 38,
    QSFP_DD_RX4_POWER_OFFSET                     = 40,
    QSFP_DD_RX5_POWER_OFFSET                     = 42,
    QSFP_DD_RX6_POWER_OFFSET                     = 44,
    QSFP_DD_RX7_POWER_OFFSET                     = 46,
    QSFP_DD_RX8_POWER_OFFSET                     = 48,
    QSFP_DD_TX1_BIAS_OFFSET                      = 50,
    QSFP_DD_TX2_BIAS_OFFSET                      = 52,
    QSFP_DD_TX3_BIAS_OFFSET                      = 54,
    QSFP_DD_TX4_BIAS_OFFSET                      = 56,
    QSFP_DD_TX5_BIAS_OFFSET                      = 58,
    QSFP_DD_TX6_BIAS_OFFSET                      = 60,
    QSFP_DD_TX7_BIAS_OFFSET                      = 62,
    QSFP_DD_TX8_BIAS_OFFSET                      = 64,
    QSFP_DD_TX1_POWER_OFFSET                     = 66,
    QSFP_DD_TX2_POWER_OFFSET                     = 68,
    QSFP_DD_TX3_POWER_OFFSET                     = 70,
    QSFP_DD_TX4_POWER_OFFSET                     = 72,
    QSFP_DD_TX5_POWER_OFFSET                     = 74,
    QSFP_DD_TX6_POWER_OFFSET                     = 76,
    QSFP_DD_TX7_POWER_OFFSET                     = 78,
    QSFP_DD_TX8_POWER_OFFSET                     = 80,
    QSFP_DD_TX_CONTROL_OFFSET                    = 84,
    QSFP_DD_TX_CDR_CONTROL_OFFSET                = 90,
    QSFP_DD_RX_CDR_CONTROL_OFFSET                = 91,

} qsfp_dd_reg_offset_t;

typedef enum {
    QSFP_DD_TEMP_HIGH_ALARM_THRESHOLD_OFFSET         = 128,
    QSFP_DD_TEMP_LOW_ALARM_THRESHOLD_OFFSET          = 130,
    QSFP_DD_TEMP_HIGH_WARNING_THRESHOLD_OFFSET       = 132,
    QSFP_DD_TEMP_LOW_WARNING_THRESHOLD_OFFSET        = 134,
    QSFP_DD_TEMP2_HIGH_ALARM_THRESHOLD_OFFSET        = 136,
    QSFP_DD_TEMP2_LOW_ALARM_THRESHOLD_OFFSET         = 138,
    QSFP_DD_TEMP2_HIGH_WARNING_THRESHOLD_OFFSET      = 140,
    QSFP_DD_TEMP2_LOW_WARNING_THRESHOLD_OFFSET       = 142,
    QSFP_DD_VOLT_HIGH_ALARM_THRESHOLD_OFFSET         = 144,
    QSFP_DD_VOLT_LOW_ALARM_THRESHOLD_OFFSET          = 146,
    QSFP_DD_VOLT_HIGH_WARNING_THRESHOLD_OFFSET       = 148,
    QSFP_DD_VOLT_LOW_WARNING_THRESHOLD_OFFSET        = 150,
    QSFP_DD_VOLT2_HIGH_ALARM_THRESHOLD_OFFSET        = 152,
    QSFP_DD_VOLT2_LOW_ALARM_THRESHOLD_OFFSET         = 154,
    QSFP_DD_VOLT2_HIGH_WARNING_THRESHOLD_OFFSET      = 156,
    QSFP_DD_VOLT2_LOW_WARNING_THRESHOLD_OFFSET       = 158,
    QSFP_DD_TEC_CURRENT_HIGH_ALARM_THRESHOLD_OFFSET  = 160,
    QSFP_DD_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET       = 176,
    QSFP_DD_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET        = 178,
    QSFP_DD_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET     = 180,
    QSFP_DD_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET      = 182,
    QSFP_DD_TX_BIAS_HIGH_ALARM_THRESHOLD_OFFSET      = 184,
    QSFP_DD_TX_BIAS_LOW_ALARM_THRESHOLD_OFFSET       = 186,
    QSFP_DD_TX_BIAS_HIGH_WARNING_THRESHOLD_OFFSET    = 188,
    QSFP_DD_TX_BIAS_LOW_WARNING_THRESHOLD_OFFSET     = 190,
    QSFP_DD_TX_PWR_HIGH_ALARM_THRESHOLD_OFFSET       = 192,
    QSFP_DD_TX_PWR_LOW_ALARM_THRESHOLD_OFFSET        = 194,
    QSFP_DD_TX_PWR_HIGH_WARNING_THRESHOLD_OFFSET     = 196,
    QSFP_DD_TX_PWR_LOW_WARNING_THRESHOLD_OFFSET      = 198,

} qsfp_dd_page3_reg_offset_t;


/* Table 18 - Status Indicators (Page A0) */
#define QSFP_FLAT_MEM_BIT_OFFSET    2

/* Table 19 — Channel status Interrupt Flags (Page A0) */
#define QSFP_TX_LOS_BIT_OFFSET  0x10
#define QSFP_RX_LOS_BIT_OFFSET  0x01
#define QSFP_DD_TX_LOS_BIT_OFFSET  0x01
#define QSFP_DD_RX_LOS_BIT_OFFSET  0x01

/* Table 20 - Module Monitor Interrupt Flags (Page A0) */
#define QSFP_TEMP_HIGH_ALARM_FLAG    (1 << 7) /* 0x80 */
#define QSFP_TEMP_LOW_ALARM_FLAG     (1 << 6) /* 0x40 */
#define QSFP_TEMP_HIGH_WARNING_FLAG  (1 << 5) /* 0x20 */
#define QSFP_TEMP_LOW_WARNING_FLAG   (1 << 4) /* 0x10 */

#define QSFP_VOLT_HIGH_ALARM_FLAG    (1 << 7) /* 0x80 */
#define QSFP_VOLT_LOW_ALARM_FLAG     (1 << 6) /* 0x40 */
#define QSFP_VOLT_HIGH_WARNING_FLAG  (1 << 5) /* 0x20 */
#define QSFP_VOLT_LOW_WARNING_FLAG   (1 << 4) /* 0x10 */

/* Table 21 — Channel Monitor Interrupt Flags (Page A0)*/
/* offset 9 and 10 */
/* QSFP_RX13_XXXX -> RX13 represents channel 1 and channel 3 of QSFP */
/* QSFP_RX24_XXXX -> RX24 represents channel 2 and channel 4 of QSFP */
#define QSFP_RX13_POWER_HIGH_ALARM_FLAG      (1 << 7) /* 0x80 */
#define QSFP_RX13_POWER_LOW_ALARM_FLAG       (1 << 6) /* 0x40 */
#define QSFP_RX13_POWER_HIGH_WARNING_FLAG    (1 << 5) /* 0x20 */
#define QSFP_RX13_POWER_LOW_WARNING_FLAG     (1 << 4) /* 0x10 */
#define QSFP_RX24_POWER_HIGH_ALARM_FLAG      (1 << 3) /* 0x08 */
#define QSFP_RX24_POWER_LOW_ALARM_FLAG       (1 << 2) /* 0x04 */
#define QSFP_RX24_POWER_HIGH_WARNING_FLAG    (1 << 1) /* 0x02 */
#define QSFP_RX24_POWER_LOW_WARNING_FLAG     (1 << 0) /* 0x01 */

/* offset 11 and 12 */
/* QSFP_TX13_XXXX -> TX13 represents channel 1 and channel 3 of QSFP */
/* QSFP_TX24_XXXX -> TX24 represents channel 2 and channel 4 of QSFP */
#define QSFP_TX13_BIAS_HIGH_ALARM_FLAG      (1 << 7) /* 0x80 */
#define QSFP_TX13_BIAS_LOW_ALARM_FLAG       (1 << 6) /* 0x40 */
#define QSFP_TX13_BIAS_HIGH_WARNING_FLAG    (1 << 5) /* 0x20 */
#define QSFP_TX13_BIAS_LOW_WARNING_FLAG     (1 << 4) /* 0x10 */
#define QSFP_TX24_BIAS_HIGH_ALARM_FLAG      (1 << 3) /* 0x08 */
#define QSFP_TX24_BIAS_LOW_ALARM_FLAG       (1 << 2) /* 0x04 */
#define QSFP_TX24_BIAS_HIGH_WARNING_FLAG    (1 << 1) /* 0x02 */
#define QSFP_TX24_BIAS_LOW_WARNING_FLAG     (1 << 0) /* 0x01 */

/* CDR support bits for TX and RX on a module
 * Option Values (Address 194) (Page 00) */
#define QSFP_TX_CDR_CONTROL_BIT_OFFSET      (7)
#define QSFP_RX_CDR_CONTROL_BIT_OFFSET      (6)

/* CDR control byte (Address 98), these macros return
 * TX and RX control bits for a particular channel. */

#define QSFP_TX_CDR_CONTROL_BIT(channel)  (channel + 4)
#define QSFP_RX_CDR_CONTROL_BIT(channel)  (channel)
#define QSFP_DD_TX_CDR_CONTROL_BIT(channel)  (channel)
#define QSFP_DD_RX_CDR_CONTROL_BIT(channel)  (channel)

/* Table 39 — Option Values (Address 195) (Page 00) */
#define QSFP_TX_DISABLE_BIT_OFFSET  4
#define QSFP_RATE_SELECT_BIT_OFFSET 5

#define QSFP_WAVELENGTH_DIVIDER             20
#define QSFP_WAVELENGTH_TOLERANCE_DIVIDER   200

#endif
