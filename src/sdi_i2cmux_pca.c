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
 * filename: sdi_i2cmux_pca.c
 */


/******************************************************************************
 * Creates an i2cmux device and registers i2c bus for every mux channel.
 * i2c-mux-pin: its an i2c multiplexer, that multiplexes one i2c bus onto 'n'
 * i2c bus based on the first byte sent to it. This device is attached to
 * pseudo bus as it has references to i2c bus (being multiplexed).
 *
 * todo:
 * 1. auto-generation of instance id.
 * 2. error handling for bus init failures.
 *****************************************************************************/

/**
 * XML Format for i2cmux pca :
 * <sdi_i2cmux_pca instance="0" parent="smbus0">
 *      <i2c instance="2" mux_addr="0x71" mux_sel_value="0x1" bus_name="smbus0_0"></i2c>
 *      <i2c instance="3" mux_addr="0x71" mux_sel_value="0x2" bus_name="smbus0_1"></i2c>
 *      <i2c instance="4" mux_addr="0x72" mux_sel_value="0x4" bus_name="smbus0_2"></i2c>
 *      <i2c instance="5" mux_addr="0x72" mux_sel_value="0x8" bus_name="smbus0_3"></i2c>
 * </sdi_i2cmux_pca>
 *
 */

#include "sdi_i2cmux_pca.h"
#include "sdi_i2c_mux_attr.h"
#include "sdi_common_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_i2c_bus_framework.h"
#include "std_utils.h"
#include "std_assert.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/**
 * sdi_i2cmux_pca_acquire_bus
 * acquire i2c mux channel bus
 * sequence of operations:
 *  1. acquire mux device lock to prevent other access to mux device
 *  2. acquire i2c bus to which this mux is attached.
 *  3. select the channel by sending the I2C byte
 * param[in] bus_handle - i2c mux channel bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pca_chan_acquire_bus(sdi_i2c_bus_hdl_t bus_handle)
{
    t_std_error error = STD_ERR_OK;
    sdi_i2cmux_pca_chan_bus_handle_t bus = (sdi_i2cmux_pca_chan_bus_handle_t) bus_handle;
    sdi_i2cmux_pca_hdl_t mux = bus->i2c_mux;

    error = std_mutex_lock(&(mux->mux_lock));
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d acquiring lock failed with error %d\n",
                              __FUNCTION__, __LINE__, error);
        return error;
    }

    do {
        /* Bus acquisition is handled by sdi_smbus_write_byte api */
        SDI_DEVICE_TRACEMSG_LOG("%s:%d sending select on bus %s addr %02x value %02x\n",
                __FUNCTION__, __LINE__, mux->i2c_bus_name,
                bus->mux_i2c_addr, bus->mux_sel_value);
        error = sdi_smbus_write_byte(mux->i2c_bus, bus->mux_i2c_addr,
                                    0, bus->mux_sel_value, 0);
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d PCA channel select failed with error %d\n",
                    __FUNCTION__, __LINE__, error);
            break;
        }
    } while(0);

    if (error != STD_ERR_OK) {
        std_mutex_unlock (&(mux->mux_lock));
    }

    /* If success, unlock is handled by sdi_i2cmux_pca_chan_release_bus API */
    return error;
}

/**
 * sdi_i2cmux_pca_chan_release_bus
 * release i2c mux channel bus
 * sequence of operations:
 * 1. send 0 to i2c mux
 * 2. release i2c mux channel bus
 * 3. release mux device lock.
 * param[in] bus_handle - i2c mux channel bus handle
 * return none
 */
static void sdi_i2cmux_pca_chan_release_bus(sdi_i2c_bus_hdl_t bus_handle)
{
    sdi_i2cmux_pca_chan_bus_handle_t bus = (sdi_i2cmux_pca_chan_bus_handle_t) bus_handle;
    sdi_i2cmux_pca_hdl_t mux = bus->i2c_mux;

    /* Deselect the mux (all channels) */
    SDI_DEVICE_TRACEMSG_LOG("%s:%d sending deselect on bus %s addr %02x\n",
            __FUNCTION__, __LINE__, mux->i2c_bus_name, bus->mux_i2c_addr);
    sdi_smbus_write_byte(mux->i2c_bus, bus->mux_i2c_addr, 0, 0, 0);
    std_mutex_unlock (&(mux->mux_lock));
}

/**
 * sdi_i2cmux_pca_chan_get_capability
 * get the capability of i2c mux channel bus
 * param[out] capability - i2c mux channel bus capability
 * return none
 */
static void sdi_i2cmux_pca_chan_get_capability
    (sdi_i2c_bus_hdl_t bus_handle, sdi_i2c_bus_capability_t *capability)
{
    sdi_i2cmux_pca_chan_bus_handle_t bus = (sdi_i2cmux_pca_chan_bus_handle_t) bus_handle;

    sdi_i2c_bus_get_capability(bus->i2c_mux->i2c_bus, capability);
}

/**
 * sdi_i2cmux_pca_chan_execute
 * execute i2c bus operation
 * param[in] bus_handle - i2c mux channel bus handle
 * param[in] address - i2c address of slave device
 * param[in] operation - i2c bus operation (read/write)
 * param[in] data_type - i2c bus transaction type (byte,word,block)
 * param[in] commandbuf - address offset
 * param[out] buffer - data read from/written to i2c slave
 * param[out] block_len - i2c block data size
 * param[in] flags - PEC
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pca_chan_execute (sdi_i2c_bus_hdl_t bus_handle,
                                                sdi_i2c_addr_t address,
                                                sdi_smbus_operation_t operation,
                                                sdi_smbus_data_type_t data_type,
                                                uint_t commandbuf, void *buffer,
                                                size_t *block_len, uint_t flags)
{
    sdi_i2cmux_pca_chan_bus_handle_t bus = (sdi_i2cmux_pca_chan_bus_handle_t) bus_handle;

    return sdi_smbus_execute(bus->i2c_mux->i2c_bus, address, operation, data_type,
                             commandbuf, buffer, block_len, flags);
}

/**
 * sdi_i2cmux_chan_bus_operations
 * SDI I2C Bus Operations for I2C MUX channel bus
 */
static sdi_i2c_bus_ops_t sdi_i2cmux_chan_bus_operations = {
    .sdi_i2c_acquire_bus = sdi_i2cmux_pca_chan_acquire_bus,
    .sdi_smbus_execute = sdi_i2cmux_pca_chan_execute,
    .sdi_i2c_release_bus = sdi_i2cmux_pca_chan_release_bus,
    .sdi_i2c_get_capability = sdi_i2cmux_pca_chan_get_capability,
};

/**
 * sdi_i2cmux_pca driver registration
 */
static t_std_error sdi_i2cmux_pca_driver_register(std_config_node_t node,
                                           void *bus_handle,
                                           sdi_device_hdl_t *device_hdl);

/**
 * sdi_i2cmux_pca driver initialization
 */
static t_std_error sdi_i2cmux_pca_driver_init(sdi_device_hdl_t device_hdl);

/**
 * sdi_i2cmux_pca_entry
 * SDI Driver Object to hold this driver's registration and initialization
 * function
 */
sdi_driver_t sdi_i2cmux_pca_entry = {
    sdi_i2cmux_pca_driver_register,
    sdi_i2cmux_pca_driver_init
};

/**
 * sdi_i2cmux_channel_init
 * Initialize i2c mux channel bus : initialize lock, i2c mux channel bus
 * operations, initialize i2c device list
 * param[in] bus_handle - i2c mux channel bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_channel_init(sdi_bus_hdl_t bus_handle)
{
    sdi_i2cmux_pca_chan_bus_handle_t i2cmux_chan_bus =
        (sdi_i2cmux_pca_chan_bus_handle_t) bus_handle;

    STD_ASSERT(i2cmux_chan_bus != NULL);

    i2cmux_chan_bus->i2c_bus.ops = &sdi_i2cmux_chan_bus_operations;

    sdi_bus_init_device_list(bus_handle);

    return STD_ERR_OK;
}

/**
 * sdi_i2cmux_pca_driver_register
 * Register i2c mux device and register every i2c mux channel on this i2c mux
 * param[in] node - i2c mux pin config node
 * param[in] bus_handle - pseudo bus handle
 * param[out] device_hdl - handle populated with i2c mux device handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pca_driver_register(std_config_node_t node,
                                           void *bus_handle,
                                           sdi_device_hdl_t *device_hdl)
{

    char *node_attr = NULL;
    sdi_bus_hdl_t bus = NULL;
    std_config_node_t cur_node = NULL;
    sdi_i2cmux_pca_chan_bus_handle_t i2cmux_chan_bus = NULL;
    sdi_device_hdl_t dev = NULL;

    STD_ASSERT(device_hdl != NULL);

    sdi_i2cmux_pca_t *i2cmux = (sdi_i2cmux_pca_t *)
        calloc (sizeof(sdi_i2cmux_pca_t), 1);

    STD_ASSERT(i2cmux != NULL);

    sdi_bus_init_list(&i2cmux->channel_list);

    dev = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev != NULL);

    dev->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev->instance = (uint_t) strtoul (node_attr, NULL, 0);

    /* dev->addr is not used by this device */

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL) {
        snprintf(dev->alias,SDI_MAX_NAME_LEN,"sdi-i2cmux-pca-%u", dev->instance);
    } else {
        safestrncpy(dev->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    dev->callbacks = &sdi_i2cmux_pca_entry;

    dev->private_data = (void *)i2cmux;

    node_attr = std_config_attr_get(node,
        SDI_DEV_ATTR_SDI_I2CMUX_PARENT_I2CBUS);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(i2cmux->i2c_bus_name, node_attr, SDI_MAX_NAME_LEN);

    *device_hdl = dev;

    for (cur_node = std_config_get_child(node); cur_node != NULL;
            cur_node = std_config_next_node(cur_node)) {

        SDI_DEVICE_TRACEMSG_LOG("%s:%d registering childnode %s\n",
                __FUNCTION__, __LINE__, std_config_name_get(cur_node));

        i2cmux_chan_bus = (sdi_i2cmux_pca_chan_bus_handle_t)
            calloc (sizeof(sdi_i2cmux_pca_chan_bus_t), 1);
        STD_ASSERT(i2cmux_chan_bus != NULL);

        node_attr = std_config_attr_get(cur_node,
            SDI_DEV_ATTR_SDI_I2CMUX_ADDRESS);
        STD_ASSERT(node_attr != NULL);
        i2cmux_chan_bus->mux_i2c_addr.i2c_addr = strtoul(node_attr, NULL, 0);

        node_attr = std_config_attr_get(cur_node,
                                        SDI_DEV_ATTR_SDI_I2CMUX_SELECT);
        STD_ASSERT(node_attr != NULL);
        i2cmux_chan_bus->mux_sel_value = (uint_t) strtoul (node_attr, NULL, 0);

        node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
        STD_ASSERT(node_attr != NULL);
        bus = (sdi_bus_hdl_t) &(i2cmux_chan_bus->i2c_bus);
        bus->bus_id = (uint_t) strtoul (node_attr, NULL, 0);

        bus->bus_type = SDI_I2C_BUS;
        bus->bus_init = sdi_i2cmux_channel_init;

        node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_BUS_NAME);
        STD_ASSERT(node_attr != NULL);
        safestrncpy(bus->bus_name, node_attr, SDI_MAX_NAME_LEN);

        i2cmux_chan_bus->i2c_mux = i2cmux;

        sdi_i2c_bus_register((sdi_i2c_bus_hdl_t) i2cmux_chan_bus);

        sdi_bus_enqueue_list(&i2cmux->channel_list,
                             (sdi_bus_hdl_t)i2cmux_chan_bus);

        sdi_bus_register_device_list(cur_node, (sdi_bus_hdl_t)i2cmux_chan_bus);

    }

    return STD_ERR_OK;
}


/**
 * sdi_i2cmux_pca_driver_init
 * Initialize i2c mux device
 * param[in] device_hdl - i2c mux device handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pca_driver_init(sdi_device_hdl_t device_hdl)
{
    t_std_error error = STD_ERR_OK;
    sdi_i2cmux_pca_hdl_t i2cmux = (sdi_i2cmux_pca_hdl_t) (device_hdl->private_data);
    sdi_i2c_bus_hdl_t i2c_bus = NULL;

    i2c_bus = sdi_get_i2c_bus_handle_by_name(i2cmux->i2c_bus_name);
    STD_ASSERT(i2c_bus != NULL);
    i2cmux->i2c_bus = i2c_bus;

    error = std_mutex_lock_init_non_recursive(&(i2cmux->mux_lock));
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d lock init failed with error %d\n",
                              __FUNCTION__, __LINE__, error);
        return error;
    }

    sdi_init_bus_for_each_bus_in_list(&i2cmux->channel_list,
                                      sdi_i2cmux_channel_init, NULL);

    return error;
}
