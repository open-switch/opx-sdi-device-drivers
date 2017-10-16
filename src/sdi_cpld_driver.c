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
 * filename: sdi_cpld_driver.c
 * Implements APIs to read/write CPLD I2C bus
 */

#include "sdi_cpld.h"
#include "sdi_host_system_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_cpld_driver_attr.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_bit_ops.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static sdi_i2c_addr_t device_i2c_addr = { .i2c_addr = 0, .addr_mode_16bit = 0 };
static t_std_error sdi_cpld_driver_register(std_config_node_t node, void *bus_handle, sdi_device_hdl_t *device_hdl);
static t_std_error sdi_cpld_driver_init(sdi_device_hdl_t device_hdl);

/*
 * Read IOM slot using CPLD I2C
 */
static t_std_error sdi_cpld_get_iom_slot_pos(sdi_device_hdl_t cpld_device, uint8_t *input_slot) {
    t_std_error rc = STD_ERR_OK;
    uint8_t slot = 0;

    device_i2c_addr.i2c_addr = MASTER_CPLD_I2C_ADDR;
    rc = sdi_smbus_read_byte(cpld_device->bus_hdl, device_i2c_addr, (SDI_CPLD_CTRL1_REGISTER_OFFSET & 0xff),
            &slot, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("cpld smbus read failed at addr : %x reg : %x for %s rc : %d",
                              cpld_device->addr, SDI_CPLD_CTRL1_REGISTER_OFFSET, cpld_device->alias, rc);
        return rc;
    }
    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset %x and #data %x\n", SDI_CPLD_CTRL1_REGISTER_OFFSET, slot);

    if (STD_BIT_TEST(slot, SDI_CPLD_CTRL1_BIT_IOM_SLOT_POS)) {
        *input_slot = 1;
    } else {
        *input_slot = 0;
    }

    return rc;
}

/*
 * Read IOM package notify using CPLD I2C
 */
static t_std_error sdi_cpld_get_iom_pkg_notify(sdi_device_hdl_t cpld_device, bool *input_presence) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     presence = 0;

    device_i2c_addr.i2c_addr = MASTER_CPLD_I2C_ADDR;
    rc = sdi_smbus_read_byte(cpld_device->bus_hdl, device_i2c_addr, (SDI_CPLD_CTRL1_REGISTER_OFFSET & 0xff),
            &presence, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("cpld smbus read failed at addr : %x reg : %x for %s rc : %d",
                              cpld_device->addr, SDI_CPLD_CTRL1_REGISTER_OFFSET, cpld_device->alias, rc);
        return rc;
    }

    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset %x and #data %d\n", SDI_CPLD_CTRL1_REGISTER_OFFSET, presence);

    if (STD_BIT_TEST(presence, SDI_CPLD_CTRL1_BIT_IOM_PACKG_NOTIFY)) {
        *input_presence = true;
    } else {
        *input_presence = false;
    }

    return rc;
}

/*
 * Write IOM booted using CPLD I2C
 */
static t_std_error sdi_cpld_set_iom_booted(sdi_device_hdl_t cpld_device, bool input_booted) {
    t_std_error rc = STD_ERR_OK;
    uint8_t booted = 0;

    device_i2c_addr.i2c_addr = MASTER_CPLD_I2C_ADDR;
    rc = sdi_smbus_read_byte(cpld_device->bus_hdl, device_i2c_addr, (SDI_CPLD_CTRL1_REGISTER_OFFSET & 0xff),
            &booted, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("cpld smbus read failed at addr : %x reg : %x for %s rc : %d",
                              cpld_device->addr, SDI_CPLD_CTRL1_REGISTER_OFFSET, cpld_device->alias, rc);
        return rc;
    }

    if (input_booted) {
        STD_BIT_SET(booted, SDI_CPLD_CTRL1_BIT_IOM_BOOTED);
    } else {
        STD_BIT_CLEAR(booted, SDI_CPLD_CTRL1_BIT_IOM_BOOTED);
    }

    rc = sdi_smbus_write_byte(cpld_device->bus_hdl, device_i2c_addr, SDI_CPLD_CTRL1_REGISTER_OFFSET, booted, SDI_I2C_FLAG_NONE);

    SDI_DEVICE_TRACEMSG_LOG("SMBUS write #offset %x and #data %x\n", SDI_CPLD_CTRL1_REGISTER_OFFSET, booted);

    return rc;
}

/* Callback handlers for FReD CPLD */
static host_system_ctrl_t cpld_ctrl = {
    .get_slot = sdi_cpld_get_iom_slot_pos,
    .get_pkg_notify = sdi_cpld_get_iom_pkg_notify,
    .set_booted = sdi_cpld_set_iom_booted
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t* sdi_cpld_driver_entry_callbacks(void) {
    /*Export Driver table*/
    static const sdi_driver_t cpld_driver_entry = {
        sdi_cpld_driver_register,
        sdi_cpld_driver_init
    };

    return &cpld_driver_entry;
}

/*
 * Register function for FReD CPLD
 * node[in]         - Device node from configuration file
 * bus_handle[in]   - Parent bus handle of the device
 * device_hdl[out]  - Device handle which is filled by this function
 * return           - t_std_error
 */
static t_std_error sdi_cpld_driver_register(std_config_node_t node, void *bus_handle,
                                            sdi_device_hdl_t *device_hdl) {
    sdi_device_hdl_t dev_hdl = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t *)bus_handle)->bus_type == SDI_I2C_BUS);

    dev_hdl = calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    dev_hdl->bus_hdl = bus_handle;
    dev_hdl->callbacks = sdi_cpld_driver_entry_callbacks();

    sdi_resource_add(SDI_RESOURCE_HOST_SYSTEM, "cpld", (void *)dev_hdl, &cpld_ctrl);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}

/*
 * Initialize the device
 * device_hdl[in] - Handle to the device
 * return         - t_std_error
 */
static t_std_error sdi_cpld_driver_init(sdi_device_hdl_t dev_hdl) {
    return STD_ERR_OK;
}




