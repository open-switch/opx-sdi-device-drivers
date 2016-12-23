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
 * filename: sdi_nvram_resource_attr.h
 */


#ifndef __SDI_NVRAM_RESOURCE_ATTR_H__
#define __SDI_NVRAM_RESOURCE_ATTR_H__

/**
 * @file sdi_nvram_resource_attr.h
 * @brief defines list of attributes commonly used for SDI_NVRAM_RESOURCE
 * configuration
 *
 * @defgroup sdi_config_nvram_resource_attributes SDI configuration NVRAM
 * resources attributes
 * @ingroup sdi_config_attributes
 * @brief Configuration Attributes commonly used by Device nodes that export
 *    @ref SDI_NVRAM_RESOURCE
 *
 * @{
 */

/**
 * @def Node name used for representing the nvram
 */
#define SDI_DEV_NODE_NVRAM  "nvram"

/**
 * @def Device (file) to use, if any; none => use chassis EEPROM
 */
#define SDI_DEV_ATTR_NVRAM_DEV  "dev"

/**
 * @def Attribute for offset (bytes) into above device where NVRAM begins
 */
#define SDI_DEV_ATTR_NVRAM_OFS  "offset"

/**
 * @def Attribute for size (bytes) of NVRAM
 */
#define SDI_DEV_ATTR_NVRAM_SIZE  "size"

/**
 * @}
 */

#endif   /* __SDI_NVRAM_RESOURCE_ATTR_H__ */
