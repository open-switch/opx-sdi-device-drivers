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
 * filename: sdi_sf_fan_attr.h
 */


/**
 * @file sdi_sf_fan_attr.h
 * @brief defines list of attributes supported for smartfusion driven fan controller
 *
 */

#ifndef __SDI_SF_FAN_ATTR_H__
#define __SDI_SF_FAN_ATTR_H__

/**
 * @def Attribute used for representing fan speed address offset
 */
#define SDI_DEV_ATTR_SF_FAN_SPEED_OFFSET                   "fan_speed_offset"

/**
 * @def Attribute used for representing fan status pin bus name
 */
#define SDI_DEV_ATTR_SF_FAN_STATUS_BUS_NAME                "fan_status_bus"


#endif   /* __SDI_SF_FAN_ATTR_H__ */
