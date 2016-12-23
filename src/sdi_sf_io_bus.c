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
 * filename: sdi_sf_io_bus.c
 */

#include "sdi_device_common.h"
#include "sdi_smartfusion_io.h"
#include "sdi_sf_io_bus.h"
#include "sdi_sf_io_bus_attr.h"
#include "sdi_io_port_api.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_assert.h"
#include "std_utils.h"

#include <stdlib.h>

t_std_error sf_bus_read_byte(sdi_sf_io_bus_hdl_t bus_hdl, uint_t addr, uint8_t *value)
{
    sf_io_bus_t *sf_io_bus = NULL;
    STD_ASSERT(bus_hdl != NULL);
    STD_ASSERT(bus_hdl->bus.bus_type == SDI_SF_IO_BUS);
    STD_ASSERT(value != NULL);

    sf_io_bus = (sf_io_bus_t *)bus_hdl;

    /* lock is acquired by user before invoking this API via sdi_sf_io_acquire_bus */
    sdi_io_port_write_byte(sf_io_bus->ram_addr_high_io_addr, ((addr >> 8) & 0xff));
    sdi_io_port_write_byte(sf_io_bus->ram_addr_low_io_addr, (addr & 0xff));
    sdi_io_port_read_byte(sf_io_bus->ram_read_data_io_addr, value);
    /* lock is released by user after invoking this API via sdi_sf_io_release_bus */
    return STD_ERR_OK;
}

t_std_error sf_bus_write_byte(sdi_sf_io_bus_hdl_t bus_hdl, uint_t addr, uint8_t value)
{
    sf_io_bus_t *sf_io_bus = NULL;
    STD_ASSERT(bus_hdl != NULL);
    STD_ASSERT(bus_hdl->bus.bus_type == SDI_SF_IO_BUS);

    sf_io_bus = (sf_io_bus_t *)bus_hdl;

    /* lock is acquired by user before invoking this API via sdi_sf_io_acquire_bus */
    sdi_io_port_write_byte(sf_io_bus->ram_addr_high_io_addr, ((addr >> 8) & 0xff));
    sdi_io_port_write_byte(sf_io_bus->ram_addr_low_io_addr, (addr & 0xff));
    sdi_io_port_write_byte(sf_io_bus->ram_write_data_io_addr, value);
    /* lock is released by user after invoking this API via sdi_sf_io_release_bus */
    return STD_ERR_OK;
}

static sdi_sf_io_bus_ops_t sf_bus_ops = {
    .sdi_sf_io_bus_read_byte = sf_bus_read_byte,
    .sdi_sf_io_bus_write_byte = sf_bus_write_byte,
};

static t_std_error sf_io_bus_driver_init(sdi_bus_hdl_t bus_hdl)
{
    t_std_error rc = STD_ERR_OK;
    sf_io_bus_t *sf_io_bus = NULL;
    STD_ASSERT(bus_hdl != NULL);
    STD_ASSERT(bus_hdl->bus_type == SDI_SF_IO_BUS);

    sf_io_bus = (sf_io_bus_t *)bus_hdl;

    rc = ioperm(sf_io_bus->ram_addr_high_io_addr, BYTE_SIZE, IO_PORT_PERM_ENABLE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d IO Permission %s failed for %x with error %d\n",
                             __FUNCTION__, __LINE__, bus_hdl->bus_name,
                             sf_io_bus->ram_addr_high_io_addr, rc);
        return rc;
    }

    rc = ioperm(sf_io_bus->ram_addr_low_io_addr, BYTE_SIZE, IO_PORT_PERM_ENABLE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d IO Permission %s failed for %x with error %d\n",
                             __FUNCTION__, __LINE__, bus_hdl->bus_name,
                             sf_io_bus->ram_addr_low_io_addr, rc);
        return rc;
    }

    rc = ioperm(sf_io_bus->ram_write_data_io_addr, BYTE_SIZE, IO_PORT_PERM_ENABLE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d IO Permission %s failed for %x with error %d\n",
                             __FUNCTION__, __LINE__, bus_hdl->bus_name,
                             sf_io_bus->ram_write_data_io_addr, rc);
        return rc;
    }

    rc = ioperm(sf_io_bus->ram_read_data_io_addr, BYTE_SIZE, IO_PORT_PERM_ENABLE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d IO Permission %s failed for %x with error %d\n",
                             __FUNCTION__, __LINE__, bus_hdl->bus_name,
                             sf_io_bus->ram_read_data_io_addr, rc);
        return rc;
    }

    sdi_bus_init_device_list(bus_hdl);

    return rc;
}

static t_std_error sf_io_bus_driver_register(std_config_node_t node,
                                             sdi_bus_hdl_t *bus_hdl)
{
    char *node_attr = NULL;
    t_std_error error = STD_ERR_OK;
    sdi_sf_io_bus_hdl_t sf_bus_hdl = NULL;
    sf_io_bus_t *sf_bus = (sf_io_bus_t *)calloc(sizeof(sf_io_bus_t), 1);

    STD_ASSERT(sf_bus != NULL);

    sf_bus_hdl = &(sf_bus->bus);
    sf_bus_hdl->bus.bus_init = sf_io_bus_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(sf_bus_hdl->bus.bus_name, node_attr, SDI_MAX_NAME_LEN);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    sf_bus_hdl->bus.bus_id = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_BUS_RAM_ADDR_HIGH);
    STD_ASSERT(node_attr != NULL);
    sf_bus->ram_addr_high_io_addr = (uint_t) strtoul (node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_BUS_RAM_ADDR_LOW);
    STD_ASSERT(node_attr != NULL);
    sf_bus->ram_addr_low_io_addr = (uint_t) strtoul (node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_BUS_RAM_READ_DATA_ADDR);
    STD_ASSERT(node_attr != NULL);
    sf_bus->ram_read_data_io_addr = (uint_t) strtoul (node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_BUS_RAM_WRITE_DATA_ADDR);
    STD_ASSERT(node_attr != NULL);
    sf_bus->ram_write_data_io_addr = (uint_t) strtoul (node_attr, NULL, 16);

    std_mutex_lock_init_non_recursive(&(sf_bus_hdl->lock));
    sf_bus_hdl->ops = &sf_bus_ops;

    sf_bus_hdl->bus.bus_type = SDI_SF_IO_BUS;

    error = sdi_bus_register((sdi_bus_hdl_t )sf_bus);
    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d registration failed %s with error %d\n",
                              __FUNCTION__, __LINE__, sf_bus_hdl->bus.bus_name, error);
        free(sf_bus);
        return error;
    }

    *bus_hdl = (sdi_bus_hdl_t ) sf_bus;

    sdi_bus_register_device_list(node, (sdi_bus_hdl_t) sf_bus);

    return error;
}

const sdi_bus_driver_t *sf_io_bus_entry_callbacks(void)
{
    /* Export bus driver table */
    static const sdi_bus_driver_t sf_io_bus_entry = {
        .bus_register = sf_io_bus_driver_register,
        .bus_init = sf_io_bus_driver_init
    };
    return &sf_io_bus_entry;
}
