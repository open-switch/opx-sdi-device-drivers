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
 * filename: sdi_sf_io_bus_attr.h
 */


/**
 * @file sdi_sf_io_bus_attr.h
 * @brief defines list of attributes supported for smartfusion io bus node
 * configuration
 *
 * @defgroup sdi_config_bus_attributes
 * @ingroup sdi_config_attributes
 * @brief SDI configuration attributes used by sdi io bus
 * @{
 */

#ifndef __SDI_SF_IO_BUS_ATTR_H__
#define __SDI_SF_IO_BUS_ATTR_H__

/**
 * @def Attribute used for representing RAM ADDR HIGH value
 */
#define SDI_DEV_ATTR_SF_BUS_RAM_ADDR_HIGH         "ram_addr_high"

/**
 * @def Attribute used for representing RAM ADDR LOW value
 */
#define SDI_DEV_ATTR_SF_BUS_RAM_ADDR_LOW          "ram_addr_low"

/**
 * @def Attribute used for representing RAM READ DATA ADDR value
 */
#define SDI_DEV_ATTR_SF_BUS_RAM_READ_DATA_ADDR    "ram_read_data_addr"

/**
 * @def Attribute used for representing RAM WRITE DATA ADDR value
 */
#define SDI_DEV_ATTR_SF_BUS_RAM_WRITE_DATA_ADDR   "ram_write_data_addr"

/**
 * @}
 */
#endif   /* __SDI_SF_IO_BUS_ATTR_H__ */
