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
 * filename: sdi_sf_entity_info_attr.h
 */


/**
 * @file sdi_sf_entity_info_attr.h
 * @brief Defines list of attributes used for reading entity information via
 * SmartFusion
 *
 * @{
 */

#ifndef __SDI_SF_ENTITY_INFO_ATTR_H__
#define __SDI_SF_ENTITY_INFO_ATTR_H__


/**
 * @def Attribute used for representing the start address of part number of entity
 */
#define SDI_SF_ENTITY_PART_NUMBER_START_ADDR      "part_number_start_addr"

/**
 * @def Attribute used for representing the end address of part number of entity
 */
#define SDI_SF_ENTITY_PART_NUMBER_END_ADDR        "part_number_end_addr"

/**
 * @def Attribute used for representing the start address of hardware revision of entity
 */
#define SDI_SF_ENTITY_HW_REVISION_START_ADDR      "hardware_revision_start_addr"

/**
 * @def Attribute used for representing the end address of hardware revision of entity
 */
#define SDI_SF_ENTITY_HW_REVISION_END_ADDR        "hardware_revision_end_addr"

/**
 * @def Attribute used for representing the start address of service tag of entity
 */
#define SDI_SF_ENTITY_SERVICE_TAG_START_ADDR      "service_tag_start_addr"

/**
 * @def Attribute used for representing the end address of service tag of entity
 */
#define SDI_SF_ENTITY_SERVICE_TAG_END_ADDR        "service_tag_end_addr"

/**
 * @def Attribute used for representing the start address of ppid of entity
 */
#define SDI_SF_ENTITY_PPID_START_ADDR             "ppid_start_addr"

/**
 * @def Attribute used for representing the end address of ppid of entity
 */
#define SDI_SF_ENTITY_PPID_END_ADDR               "ppid_end_addr"

/**
 * @def Attribute used for representing the airflow status pin bus name
 */
#define SDI_SF_ENTITY_AIR_FLOW_STATUS             "air_flow_bus_name"

/**
 * @def Attribute used for representing the AC/DC current status pin bus name
 */
#define SDI_SF_ENTITY_CURRENT_TYPE                "current_type_bus_name"

/**
 * @def Attribute used for representing the start address of Manufacturing ID of entity
 */
#define SDI_SF_ENTITY_MFG_ID_START_ADDR           "mfg_id_start_addr"

/**
 * @def Attribute used for representing the end address of Manufacturing ID of entity
 */
#define SDI_SF_ENTITY_MFG_ID_END_ADDR             "mfg_id_end_addr"

/**
 * @def Attribute used for representing the start address of Manufacturing Date of entity
 */
#define SDI_SF_ENTITY_MFG_DATE_START_ADDR         "mfg_date_start_addr"

/**
 * @def Attribute used for representing the end address of Manufacturing Date of entity
 */
#define SDI_SF_ENTITY_MFG_DATE_END_ADDR           "mfg_date_end_addr"

/**
 * @def Attribute used for representing the start address of Serial Number of entity
 */
#define SDI_SF_ENTITY_SERIAL_NUMBER_START_ADDR    "serial_number_start_addr"

/**
 * @def Attribute used for representing the end address of Serial Number of entity
 */
#define SDI_SF_ENTITY_SERIAL_NUMBER_END_ADDR      "serial_number_end_addr"

/**
 * @def Attribute used for representing the start address of Country Code of entity
 */
#define SDI_SF_ENTITY_COUNTRY_CODE_START_ADDR     "country_code_start_addr"

/**
 * @def Attribute used for representing the end address of Country Code of entity
 */
#define SDI_SF_ENTITY_COUNTRY_CODE_END_ADDR       "country_code_end_addr"

/**
 * @}
 */

#endif /* __SDI_SF_ENTITY_INFO_ATTR_H__ */
