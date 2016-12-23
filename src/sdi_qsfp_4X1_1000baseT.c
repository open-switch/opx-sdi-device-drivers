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
 * filename: sdi_qsfp_4X1_1000baseT.c
 */


/******************************************************************************
 * sdi_qsfp_4X1_1000baseT.c
 * Implements the driver for QSFP_4X1_1000BASE Quad Small Form-factor
 * Pluggable (QSFP)
 *
 *****************************************************************************/
#include "sdi_device_common.h"
#include "sdi_pin_group_bus_framework.h"
#include "sdi_qsfp.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_media_phy_mgmt.h"
#include "std_error_codes.h"
#include "std_assert.h"

#define QSFP_4X1_1000BASE_MUX_ADDR      (0x74)
#define QSFP_4X1_1000BASE_MUX_OFFSET    (0x0)
#define QSFP_4X1_1000BASE_MAX_CHANNEl   (4)

uint_t channel_sel_mux_val [QSFP_4X1_1000BASE_MAX_CHANNEl] = {0x1, 0x2, 0x4, 0x8};

/*
 * sdi_qsfp_4X1_1000baseT_channel_select is to select the specified channel on
 * QSFP MUX.
 */

static t_std_error sdi_qsfp_4X1_1000baseT_channel_select (sdi_device_hdl_t qsfp_device,
                                                          uint_t channel)
{
    t_std_error rc = STD_ERR_OK;
    sdi_i2c_addr_t i2c_addr = { .i2c_addr = QSFP_4X1_1000BASE_MUX_ADDR, .addr_mode_16bit = 0 };


     if (channel >= QSFP_4X1_1000BASE_MAX_CHANNEl) {
         return SDI_DEVICE_ERR_PARAM;
     }

     rc = sdi_smbus_write_byte(qsfp_device->bus_hdl, i2c_addr,
                               QSFP_4X1_1000BASE_MUX_OFFSET,
                               channel_sel_mux_val[channel], SDI_I2C_FLAG_NONE);

     if (rc != STD_ERR_OK) {
         SDI_DEVICE_ERRMSG_LOG("smbus write failed at addr : %x reg : %x for %s rc : %d",
                 QSFP_4X1_1000BASE_MUX_ADDR, QSFP_4X1_1000BASE_MUX_OFFSET,
                 qsfp_device->alias, rc);
     }

     return rc;

}

/*
 * sdi_qsfp_4X1_1000baseT_mode_set is to set the mode on remote copper phy
 * for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_mode_set (sdi_device_hdl_t qsfp_device,
                                             uint_t channel, sdi_media_mode_t mode)
{
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(qsfp_device != NULL);

    do {
        rc = sdi_qsfp_4X1_1000baseT_channel_select(qsfp_device, channel);
        if (rc != STD_ERR_OK) {
            break;
        }

        rc = sdi_cusfp_phy_mode_set(qsfp_device, mode);
        if (rc != STD_ERR_OK) {
            break;
        }

    } while (0);

    return rc;
}

/*
 * sdi_qsfp_4X1_1000baseT_speed_set is to set the supported speed on remote
 * copper phy for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_speed_set (sdi_device_hdl_t qsfp_device,
                                             uint_t channel, sdi_media_speed_t speed)
{
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(qsfp_device != NULL);

    do {
        rc = sdi_qsfp_4X1_1000baseT_channel_select(qsfp_device, channel);
        if (rc != STD_ERR_OK) {
            break;
        }

        rc = sdi_cusfp_phy_speed_set(qsfp_device, speed);
        if (rc != STD_ERR_OK) {
            break;
        }
    } while (0);

    return rc;
}

/*
 * sdi_qsfp_4X1_1000baseT_autoneg_set is to set the autoneg on remote
 * copper phy for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_autoneg_set (sdi_device_hdl_t qsfp_device,
                                                uint_t channel, bool enable)
{
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(qsfp_device != NULL);

    do {
        rc = sdi_qsfp_4X1_1000baseT_channel_select(qsfp_device, channel);
        if (rc != STD_ERR_OK) {
            break;
        }

        rc = sdi_cusfp_phy_autoneg_set(qsfp_device, enable);
        if (rc != STD_ERR_OK) {
            break;
        }
    } while(0);

    return rc;
}


