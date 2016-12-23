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
 * filename: sdi_sf_fan.c
 */


/******************************************************************************
 * Implements the driver for fan controller accessed through smart fusion.
 *
 ******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_fan_internal.h"
#include "sdi_common_attr.h"
#include "sdi_fan_resource_attr.h"
#include "sdi_smartfusion_io_bus_api.h"
#include "std_assert.h"
#include "std_utils.h"
#include "sdi_device_common.h"
#include "sdi_sf_fan_attr.h"
#include "sdi_bus.h"
#include "sdi_pin.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_bus_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SF_FAN_COUNT    16

/*
 * Private data of Fans accessed through Smart Fusion
 */
typedef struct sdi_sf_fan_data {
    sdi_pin_bus_hdl_t sdi_sf_fan_status_hdl;
    uint_t fan_speed_offset;
    char *alias;
} sdi_sf_fan_data_t;

/*
 * Private data of Fan controller device accessed through Smart Fusion
 */
typedef struct sdi_sf_fan_device
{
    sdi_sf_fan_data_t fan_data[MAX_SF_FAN_COUNT];
} sdi_sf_fan_device_t;

/*
 * Resource handle for SF Fan device
 */
typedef struct sdi_sf_fan_resource_hdl
{
    sdi_device_hdl_t sdi_sf_fan_hdl;
    uint_t fan_id;
} sdi_sf_fan_resource_hdl_t;

/*Register and chip init function declarations for the sdi_sf_fan driver*/
static t_std_error sdi_sf_fan_register(std_config_node_t node, void *bus_handle,
                                sdi_device_hdl_t* device_hdl);

static t_std_error sdi_sf_fan_chip_init(sdi_device_hdl_t device_hdl);

/*
 * Callback function to retrieve the speed of the fan referred by resource
 * resource_hdl[in] - chip instance is passed as a callback data
 * speed[out]       - pointer to a buffer to get the fan speed
 * Return           - STD_ERR_OK for success or error in case of failure
 */
static t_std_error sdi_sf_fan_speed_get(void *resource_hdl, uint_t *speed)
{
    uint8_t low_byte = 0, high_byte = 0;
    sdi_device_hdl_t chip = NULL;
    t_std_error rc = STD_ERR_OK;
    sdi_sf_fan_device_t *fan_device = NULL;
    uint_t offset = 0;
    uint_t fan_id = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(speed != NULL);

    chip = ((sdi_sf_fan_resource_hdl_t*)resource_hdl)->sdi_sf_fan_hdl;
    STD_ASSERT(chip != NULL);

    fan_device = (sdi_sf_fan_device_t *)chip->private_data;
    STD_ASSERT(fan_device != NULL);

    fan_id = ((sdi_sf_fan_resource_hdl_t*)resource_hdl)->fan_id;

    offset = fan_device->fan_data[fan_id].fan_speed_offset;

    rc = sdi_sf_io_acquire_bus(chip->bus_hdl);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io acquire bus failed with rc %d\n", rc);
        return rc;
    }
    rc = sdi_sf_io_bus_read_byte(chip->bus_hdl, offset, &high_byte);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io bus read high byte failed with rc %d\n", rc);
        return rc;
    }
    rc = sdi_sf_io_bus_read_byte(chip->bus_hdl,(offset + 1), &low_byte);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io bus read low byte failed with rc %d\n", rc);
        return rc;
    }

    sdi_sf_io_release_bus(chip->bus_hdl);

    *speed = (int)((high_byte << 8) | (low_byte));

    return rc;
}

/*
 * Callback function to retrieve the fault status of the fan referred by resource
 * it will check the stall, spin, drive fail status
 * Parameters:
 * resource_hdl[in] - callback data for this function
 * status[out]      - pointer to a buffer to get the fault status of the fan.
 *                    will be true, if any fault.otherwise false.
 * Return           - STD_ERR_OK for success or the error code in case of failure
 */
static t_std_error sdi_sf_fan_status_get(void *resource_hdl, bool *status)
{
    sdi_device_hdl_t chip = NULL;
    sdi_pin_bus_hdl_t sdi_sf_fan_status_hdl = NULL;
    t_std_error rc = STD_ERR_OK;
    sdi_sf_fan_device_t *fan_device = NULL;
    sdi_pin_bus_level_t level = SDI_PIN_LEVEL_LOW;
    uint_t fan_id = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    chip = ((sdi_sf_fan_resource_hdl_t*)resource_hdl)->sdi_sf_fan_hdl;
    STD_ASSERT(chip != NULL);

    fan_device = (sdi_sf_fan_device_t *)chip->private_data;
    STD_ASSERT(fan_device != NULL);

    fan_id = ((sdi_sf_fan_resource_hdl_t*)resource_hdl)->fan_id;

    sdi_sf_fan_status_hdl = fan_device->fan_data[fan_id].sdi_sf_fan_status_hdl;
    STD_ASSERT(sdi_sf_fan_status_hdl != NULL);
    rc = sdi_pin_read_level(sdi_sf_fan_status_hdl, &level);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_fan status read failed with rc %d\n", rc);
    }
    if (level == SDI_PIN_LEVEL_HIGH) {
        *status = false;
    } else {
        *status = true;
    }
    return rc;
}

fan_ctrl_t sdi_sf_fan_resource = {
    NULL, /* no fan specific init */
    sdi_sf_fan_speed_get,
    NULL,
    sdi_sf_fan_status_get
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t *sdi_sf_fan_entry_callbacks(void)
{
    /* Export Driver table */
    static const sdi_driver_t sdi_sf_fan_entry = {
        sdi_sf_fan_register,
        sdi_sf_fan_chip_init
    };
    return &sdi_sf_fan_entry;
};

/*
 * Creates the resource handle for a specific fan.
 * dev_hdl[in] - sdi sf fan device handle
 * fan_id[in]  - id for the specific fan
 * Return sdi_sf_fan_data_t - resource handle for the specific fan
 */
static sdi_sf_fan_resource_hdl_t *sdi_sf_fan_create_resource_hdl(
                                  sdi_device_hdl_t dev_hdl, uint_t fan_id)
{
    sdi_sf_fan_resource_hdl_t *fan_resource = NULL;

    STD_ASSERT(dev_hdl != NULL);

    fan_resource = calloc(1, sizeof(sdi_sf_fan_resource_hdl_t));
    STD_ASSERT(fan_resource != NULL);

    fan_resource->sdi_sf_fan_hdl = dev_hdl;
    fan_resource->fan_id = fan_id;

    return fan_resource;
}

/*
 * Update the database for a specific fan.
 * cur_node[in]   - node to be initialised.
 * device_hdl[in] - device handle of sf fan device
 * Return - None
 */
static void sdi_sf_fan_device_database_init(std_config_node_t cur_node,
                                            void* device_hdl)
{
    sdi_sf_fan_device_t *fan_device = NULL;
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    uint_t fan_id = 0;

    STD_ASSERT(device_hdl != NULL);

    chip = (sdi_device_hdl_t)device_hdl;
    STD_ASSERT(chip != NULL);

    fan_device = (sdi_sf_fan_device_t *)chip->private_data;
    STD_ASSERT(fan_device != NULL);

    if (strncmp(std_config_name_get(cur_node), SDI_DEV_NODE_FAN,
                                   (sizeof(SDI_DEV_NODE_FAN) - 1)) != 0) {
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
    if(node_attr != NULL) {
        fan_id = (uint_t) strtoul(node_attr, NULL, 0);
    }
    STD_ASSERT(fan_id < MAX_SF_FAN_COUNT);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_SF_FAN_SPEED_OFFSET);
    STD_ASSERT(node_attr != NULL);
    fan_device->fan_data[fan_id].fan_speed_offset = (uint_t) strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_SF_FAN_STATUS_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    fan_device->fan_data[fan_id].sdi_sf_fan_status_hdl = sdi_get_pin_bus_handle_by_name(node_attr);
    STD_ASSERT(fan_device->fan_data[fan_id].sdi_sf_fan_status_hdl != NULL);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_ALIAS);
    fan_device->fan_data[fan_id].alias = calloc(1, SDI_MAX_NAME_LEN);
    STD_ASSERT(fan_device->fan_data[fan_id].alias != NULL);
    if(node_attr == NULL) {
        snprintf(fan_device->fan_data[fan_id].alias, SDI_MAX_NAME_LEN,
                 "sf-fan-%d-%d", chip->instance, fan_id);
    } else {
        snprintf(fan_device->fan_data[fan_id].alias, SDI_MAX_NAME_LEN,
                 node_attr, (strlen(node_attr)+1));
    }

    sdi_resource_add(SDI_RESOURCE_FAN, fan_device->fan_data[fan_id].alias,
                     sdi_sf_fan_create_resource_hdl(chip, fan_id), &sdi_sf_fan_resource);

}

/*
 * The config file format will be as below for sdi_sf_tmp devices
 *
 * <sdi_sf_tmp instance=<chip_instance>>
 * Mandatory attributes    : instance
 * <fan instance=<chip_instance> fan_speed_offset=<addr_offset>
 * fan_status_bus=<pin_bus_name>  alias=<alias>/>
 * </sdi_sf_tmp>
 * Mandatory attributes    : instance, fan_speed_offset, fan_status_bus and alias
 */
 /*
 * This the call back function for the device registration
 * [in] node - Config node for the device
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - pointer to the device handle which will be updated by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table fp requires it
 */
static t_std_error sdi_sf_fan_register(std_config_node_t node, void *bus_handle,
                                sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    sdi_sf_fan_device_t *fan_device = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_SF_IO_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);

    fan_device = calloc(sizeof(sdi_sf_fan_device_t),1);
    STD_ASSERT(fan_device != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL) {
        snprintf(chip->alias,SDI_MAX_NAME_LEN,"sdi_sf_fan_conroller-%d", chip->instance );
    } else {
        safestrncpy(chip->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    chip->callbacks = sdi_sf_fan_entry_callbacks();
    chip->private_data = (void *)fan_device;

    std_config_for_each_node(node, sdi_sf_fan_device_database_init, chip);

    *device_hdl = chip;

    return STD_ERR_OK;
}


static t_std_error sdi_sf_fan_chip_init(sdi_device_hdl_t device_hdl)
{
    return STD_ERR_OK;
}
