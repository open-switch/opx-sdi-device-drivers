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
 * filename: sdi_cpld_driver_attr.h
 * defines list of attributes used for Host System CPLD access
 */

#ifndef _SDI_CPLD_DRIVER_ATTR_H_
#define _SDI_CPLD_DRIVER_ATTR_H_

#define MASTER_CPLD_I2C_ADDR                                      0x33
#define SYSTEM_CPLD_I2C_ADDR                                      0x31

#define SDI_CPLD_CTRL1_REGISTER_OFFSET                            0x12
#define SDI_CPLD_CTRL1_REGISTER_SIZE                              0x01

#define SDI_CPLD_CTRL1_BIT_IOM_SLOT_POS                              1
#define SDI_CPLD_CTRL1_BIT_IOM_BOOTED                                3
#define SDI_CPLD_CTRL1_BIT_IOM_PACKG_NOTIFY                          5
#define SDI_CPLD_CTRL1_BIT_IOM_EVENT_NOTIFY                          6

#endif   /* _SDI_CPLD_DRIVER_ATTR_H_ */
