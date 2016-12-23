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
 * filename: sdi_sf_tmp_attr.h
 */


/**
 * @file sdi_sf_tmp_attr.h
 * @brief defines list of attributes supported for smartfusion driven temperature
 * sensor
 *
 */

#ifndef __SDI_SF_TMP_ATTR_H__
#define __SDI_SF_TMP_ATTR_H__

/**
 * @def Attribute used for representing Temperature sensor value address offset
 */
#define SDI_DEV_ATTR_SF_TMP_SENSOR_VALUE_OFFSET            "temp_value_offset"

/**
 * @def Attribute used for representing Temperature sensor fault status address offset
 */
#define SDI_DEV_ATTR_SF_TMP_SENSOR_FAULT_STATUS_OFFSET     "temp_fault_status_offset"

/**
 * @def Attribute used for representing Temperature sensor low threshold address offset
 */
#define SDI_DEV_ATTR_SF_TMP_SENSOR_LOW_THRESHOLD_OFFSET    "temp_low_threshold_offset"

/**
 * @def Attribute used for representing Temperature sensor low threshold address offset
 */
#define SDI_DEV_ATTR_SF_TMP_SENSOR_HIGH_THRESHOLD_OFFSET   "temp_high_threshold_offset"

#endif   /* __SDI_SF_TMP_ATTR_H__ */
