/*
 * Copyright (c) 2017 Dell Inc.
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
 * filename: sdi_comm_dev_attr.h
 * defines list of attributes used for comm_dev access
 */

#ifndef _SDI_COMM_DEV_ATTR_H_
#define _SDI_COMM_DEV_ATTR_H_

/*
 * SDI_COMM_DEV_FILE_AND_REGISTER_ACCESS_MAP
 */

// FILE ACCESS
#define SDI_COMM_DEV_TLVINFO_FILE_START_ADDRESS                 0x0000    /* READ/WRITE */
#define SDI_COMM_DEV_TLVINFO_FILE_SIZE                            1024
#define SDI_COMM_DEV_FRU_FILE_START_ADDRESS                     0x0400    /* READ/WRITE */
#define SDI_COMM_DEV_FRU_FILE_SIZE                                1024
#define SDI_COMM_DEV_DESCRIPTOR_FILE_START_ADDRESS              0x0800    /* READ/WRITE */
#define SDI_COMM_DEV_DESCRIPTOR_FILE_SIZE                         4096
#define SDI_COMM_DEV_SVG_FILE_START_ADDRESS                     0x2000    /* READ/WRITE */
#define SDI_COMM_DEV_SVG_FILE_SIZE                               21504

// REGISTER ACCESS
#define SDI_COMM_DEV_CHASSIS_SERVICE_TAG                        0x7402    /* READ ONLY */
#define SDI_COMM_DEV_CHASSIS_SERVICE_TAG_SIZE                       34
#define SDI_COMM_DEV_TLVINFO_DATA_BYTES                         0x7424    /* READ ONLY */
#define SDI_COMM_DEV_TLVINFO_DATA_BYTES_SIZE                         2
#define SDI_COMM_DEV_FRU_DATA_BYTES                             0x7426    /* READ ONLY */
#define SDI_COMM_DEV_FRU_DATA_BYTES_SIZE                             2
#define SDI_COMM_DEV_DESCRIPTOR_DATA_BYTES                      0x7428    /* READ ONLY */
#define SDI_COMM_DEV_DESCRIPTOR_DATA_BYTES_SIZE                      2
#define SDI_COMM_DEV_SVG_DATA_BYTES                             0x742C    /* READ ONLY */
#define SDI_COMM_DEV_SVG_DATA_BYTES_SIZE                             2
#define SDI_COMM_DEV_IOM_VENDOR_FW_REV                          0x7800    /* READ/WRITE */
#define SDI_COMM_DEV_IOM_VENDOR_FW_REV_SIZE                         32
#define SDI_COMM_DEV_SENSOR_STATUS                              0x7C00    /* READ/WRITE */
#define SDI_COMM_DEV_SENSOR_STATUS_SIZE                              4
#define SDI_COMM_DEV_SENSOR_TELEMETRY_TYPE                      0x7C04    /* READ/WRITE */
#define SDI_COMM_DEV_SENSOR_TELEMETRY_TYPE_SIZE                      8
#define SDI_COMM_DEV_TEMPERATURE_SENSOR_THRESHOLDS              0x7C0C    /* READ/WRITE */
#define SDI_COMM_DEV_TEMPERATURE_SENSOR_THRESHOLDS_SIZE             96
#define SDI_COMM_DEV_SENSOR_TELEMETRY                           0x7C6C    /* READ/WRITE */
#define SDI_COMM_DEV_SENSOR_TELEMETRY_SIZE                          34
#define SDI_COMM_DEV_SENSOR_06_TELEMETRY                        0x7C7A    /* READ/WRITE */
#define SDI_COMM_DEV_SENSOR_06_TELEMETRY_SIZE                        2
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_MGMT_CONTROL           0X7C8E    /* READ/WRITE */
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_MGMT_CONTROL_SIZE           2
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER        0x7C90    /* READ/WRITE */
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER_SIZE        2
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL        0x7C92    /* READ/WRITE */
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE        2
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_TIME_THRESHOLD          0x7C94    /* READ/WRITE */
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_TIME_THRESHOLD_SIZE          2
#define SDI_COMM_DEV_DELL_MANAGEMENT_FW_REVS                    0x7C96    /* READ ONLY */
#define SDI_COMM_DEV_DELL_MANAGEMENT_FW_REVS_SIZE                   64
#define SDI_COMM_DEV_IOM_SLOT_OCCUPATION                        0x7CD6    /* READ ONLY */
#define SDI_COMM_DEV_IOM_SLOT_OCCUPATION_SIZE                        2
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_STATUS                  0x7CD8    /* READ ONLY */
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_STATUS_SIZE                  4
#define SDI_COMM_DEV_DOWNLINK_PORT_ENABLE_DISABLE_REQUEST       0x7CDC    /* READ ONLY */
#define SDI_COMM_DEV_DOWNLINK_PORT_ENABLE_DISABLE_REQUEST_SIZE      16
#define SDI_COMM_DEV_IOM_STATUS_REGISTER                        0x7CFC    /* READ ONLY */
#define SDI_COMM_DEV_IOM_STATUS_REGISTER_SIZE                        2

// INTERNAL REGISTERS
#define SDI_COMM_DEV_GPIO_STATES_REGISTER                       0x8400    /* READ ONLY */
#define SDI_COMM_DEV_GPIO_STATES_REGISTER_SIZE                       2
#define SDI_COMM_DEV_EC_COMM_DEV_MGMT_CONTROL                   0x8402    /* READ ONLY */
#define SDI_COMM_DEV_EC_COMM_DEV_MGMT_CONTROL_SIZE                   1
#define SDI_COMM_DEV_MODULE_LED_CONTROL                         0x8403    /* READ ONLY */
#define SDI_COMM_DEV_MODULE_LED_CONTROL_SIZE                         1
#define SDI_COMM_DEV_MAIN_POWER_STATUS_AND_CONTROL              0x8404    /* READ ONLY */
#define SDI_COMM_DEV_MAIN_POWER_STATUS_AND_CONTROL_SIZE              1
#define SDI_COMM_DEV_SOUTHBOUND_MAILBOX_STATUS                  0x8405    /* READ ONLY */
#define SDI_COMM_DEV_SOUTHBOUND_MAILBOX_STATUS_SIZE                  4
#define SDI_COMM_DEV_EC_MAILBOX_CONTROL                         0x8409    /* READ ONLY */
#define SDI_COMM_DEV_EC_MAILBOX_CONTROL_SIZE                         4
#define SDI_COMM_DEV_DF_HASH                                    0x840D    /* READ ONLY */
#define SDI_COMM_DEV_DF_HASH_SIZE                                    8
#define SDI_COMM_DEV_SVG_HASH                                   0x8415    /* READ ONLY */
#define SDI_COMM_DEV_SVG_HASH_SIZE                                   8

// MANUFACTURING MODE REGISTERS
#define SDI_COMM_DEV_MANUFACTURING_MODE_STATUS                  0x84E1    /* READ ONLY  */
#define SDI_COMM_DEV_MANUFACTURING_MODE_STATUS_SIZE                  1
#define SDI_COMM_DEV_MANUFACTURING_MODE_NV_CONTROL              0x84E2    /* READ/WRITE */
#define SDI_COMM_DEV_MANUFACTURING_MODE_NV_CONTROL_SIZE              1
#define SDI_COMM_DEV_MANUFACTURING_MODE_INTERFACE_CONTROL       0x84E3    /* READ/WRITE */
#define SDI_COMM_DEV_MANUFACTURING_MODE_INTERFACE_CONTROL_SIZE       2

// MAILBOX BUFFERS
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_START_ADDRESS           0x8500    /* READ/WRITE */
#define SDI_COMM_DEV_NORTHBOUND_MAILBOX_SIZE                      4100
#define SDI_COMM_DEV_SOUTHBOUND_MAILBOX_START_ADDRESS           0x9504    /* READ ONLY  */
#define SDI_COMM_DEV_SOUTHBOUND_MAILBOX_SIZE                      4100

// USED IN VENDOR INTELLIGENCE MAILBOX CONTROL 0X7C92
#define SDI_COMM_DEV_BIT_NB_FLUSH_MAILBOX                            0
#define SDI_COMM_DEV_BIT_SB_PACKAGE_READ_AND_VERIFIED                1
#define SDI_COMM_DEV_BIT_NB_PACKAGE_DOWNLOAD_COMPLETE                2

// USED IN NORTHBOUND MAILBOX STATUS 0X7CD8
#define SDI_COMM_DEV_BIT_NB_PACKAGE_IN_MAILBOX                       0

// USED IN COMM DEV STATUS REGISTER
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_RESET                 0
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_PORT_NOTIFICATION     1
#define SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_NB_TMIEOUT_ALERT      2

// MISCELLANEOUS
#define COMM_DEV_I2C_ADDR                                         0x32
#define SDI_COMM_DEV_MAILBOX_HEADER_SIZE                             4

typedef struct sdi_mailbox_header {
    uint16_t checksum;
    uint16_t size;
} sdi_mailbox_header_t;

#endif   /* _SDI_COMM_DEV_ATTR_H_ */
