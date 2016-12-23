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
 * filename: sdi_device_common.h
 */


/******************************************************************************
 * @file sdi_device_common.h
 * @brief   Common header file sdi-device-driver module
 *****************************************************************************/
#ifndef __SDI_DEVICE_COMMON
#define __SDI_DEVICE_COMMON
#include "std_error_codes.h"
#include "event_log.h"

/**
  * @def Attribute used to define the maximum key size
  */
#define SDI_DEVICE_MAX_KEY_SIZE 100

#define SDI_DEVICE_ERRNO_LOG() \
    EV_LOGGING(BOARD, ERR, __func__, "errno = %d", errno)

#define SDI_DEVICE_ERRMSG_LOG(format, ...) \
    EV_LOGGING(BOARD, ERR, __func__, format, ## __VA_ARGS__)

#define SDI_DEVICE_TRACEMSG_LOG(format, ...) \
    EV_LOGGING(BOARD, DEBUG, "", format, ## __VA_ARGS__)

/**
 * @def Attribute used to map driver return codes to standard error numbers in fail cases
 */
#define SDI_DEVICE_ERRNO                   STD_ERR_MK(e_std_err_BOARD, e_std_err_code_FAIL, errno)
/**
 * @def Attribute used to map SDI sub-system return codes with standard error numbers in
 * cases of invalid permissions and functinality not supported
 */
#define SDI_DEVICE_ERRCODE(errcode)        STD_ERR_MK(e_std_err_BOARD, e_std_err_code_FAIL, errcode)
/**
 * @def Attribute used to log an error when a wrong parameter is passed
 */
#define SDI_DEVICE_ERR_PARAM               STD_ERR_MK(e_std_err_BOARD, e_std_err_code_PARAM, 0)
/**
 * @def Attribute used to log an error when a bad configuration is encountered
 */
#define SDI_DEVICE_ERR_CFG                 STD_ERR_MK(e_std_err_BOARD, e_std_err_code_CFG, 0)
/**
 * @def Attribute used to define sysfs path
 */
#define SYSFS_PATH      "/sys"

#endif /* __SDI_DEVICE_COMMON */
