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
 * filename: sdi_sf_tmp.c
 */


/******************************************************************************
 * Implements the driver for temperature sensor chip accessed through smart fusion.
 * This chip has only one sensor, so it exports only one resource to the
 * resource framework.The chip provides register interface for setting high
 * and low temperature threshold limits
 *
 ******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_smartfusion_io_bus_api.h"
#include "std_assert.h"
#include "std_utils.h"
#include "sdi_device_common.h"
#include "sdi_thermal_internal.h"
#include "sdi_temperature_resource_attr.h"
#include "sdi_sf_tmp_attr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SMF_TMP_REGISTER_ONE_BYTE_WIDTH    (1)
#define SMF_TMP_REGISTER_TWO_BYTE_WIDTH    (2)

/*
 * Smartfusion Temperature sensors device private data
 */
typedef struct sdi_sf_tmp_device
{
    uint16_t temp_sensor_addr_offset;
    uint16_t temp_low_threshold_addr_offset;
    uint16_t temp_high_threshold_addr_offset;
    uint16_t temp_status_addr_offset;

    /* Sensor threshold limits */
    int low_threshold;
    int high_threshold;
    /* Default sensor limits */
    int default_low_threshold;
    int default_high_threshold;

    bool low_threshold_offset_valid;
    bool high_threshold_offset_valid;
    bool status_offset_valid;

} sdi_sf_tmp_device_t;

/*Register and chip init function declarations for the sdi_sf_tmp driver*/
t_std_error sdi_sf_tmp_register(std_config_node_t node,void *bus_handle,sdi_device_hdl_t* device_hdl);
t_std_error sdi_sf_tmp_chip_init(sdi_device_hdl_t device_hdl);

static t_std_error sdi_sf_tmp_data_get(void *resource_hdl, uint16_t offset,
                                       uint_t width, int *data)
{
    uint8_t low_byte_tmp_value = 0, high_byte_tmp_value = 0;
    sdi_device_hdl_t chip = NULL;
    sdi_sf_tmp_device_t *tmp_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(data != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp_data = (sdi_sf_tmp_device_t*)chip->private_data;
    STD_ASSERT(tmp_data != NULL);

    rc = sdi_sf_io_acquire_bus(chip->bus_hdl);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io acquire bus failed with rc %d\n", rc);
        return rc;
    }
    rc = sdi_sf_io_bus_read_byte(chip->bus_hdl, offset, &high_byte_tmp_value);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io bus read high byte failed with rc %d\n", rc);
        return rc;
    }
    if (width == 2) {
        rc = sdi_sf_io_bus_read_byte(chip->bus_hdl,(offset + 1), &low_byte_tmp_value);
        if(rc != STD_ERR_OK)
        {
            SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io bus read low byte failed with rc %d\n", rc);
            return rc;
        }
    }

    sdi_sf_io_release_bus(chip->bus_hdl);

    if ((low_byte_tmp_value == 0xff) && (high_byte_tmp_value == 0xff)) {
        rc = SDI_DEVICE_ERRCODE(EINVAL);
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp sf io bus tmp data not ready, error %d\n", rc);
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    *data = (int)((high_byte_tmp_value << 8) | (low_byte_tmp_value));

    return rc;
}

/*
 * Retrieve temperature of the chip refered by resource.
 * This is a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [out] temperature - pointer to a buffer to ge the temperature of the chip.
 * Return - STD_ERR_OK for success and the respective error code from i2c api in case of failure
 */
static t_std_error sdi_sf_tmp_temperature_get(void *resource_hdl, int *temperature)
{
    sdi_device_hdl_t chip = NULL;
    sdi_sf_tmp_device_t *tmp_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp_data = (sdi_sf_tmp_device_t*)chip->private_data;
    STD_ASSERT(tmp_data != NULL);

    rc = sdi_sf_tmp_data_get(resource_hdl, tmp_data->temp_sensor_addr_offset,
                             SMF_TMP_REGISTER_TWO_BYTE_WIDTH, temperature);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp get temperature sensor value failed with rc %d\n", rc);
        return rc;
    }

    *temperature /= 10;

    return rc;
}

/*
 * Retrieve threshold values of the chip refered by resource.
 * This is a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [out] temperature - pointer to a buffer to get the temperature threshold value of the chip.
 * Return - STD_ERR_OK for success or EOPNOTSUPP in case of not supported threshold types
 */
static t_std_error sdi_sf_tmp_threshold_get(void *resource_hdl, sdi_threshold_t type, int *temperature)
{
    sdi_device_hdl_t chip = NULL;
    sdi_sf_tmp_device_t *tmp_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp_data = (sdi_sf_tmp_device_t*)chip->private_data;
    STD_ASSERT(tmp_data != NULL);

    switch(type)
    {
        case SDI_LOW_THRESHOLD:
            if (tmp_data->low_threshold_offset_valid == true) {
                rc = sdi_sf_tmp_data_get(resource_hdl, tmp_data->temp_low_threshold_addr_offset,
                                         SMF_TMP_REGISTER_TWO_BYTE_WIDTH, temperature);
                if(rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp get temperature low threshold failed with rc %d\n", rc);
                    return rc;
                }
            } else {
                return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
            }
            break;

        case SDI_HIGH_THRESHOLD:
            if (tmp_data->high_threshold_offset_valid == true) {
                rc = sdi_sf_tmp_data_get(resource_hdl, tmp_data->temp_high_threshold_addr_offset,
                                         SMF_TMP_REGISTER_TWO_BYTE_WIDTH, temperature);
                if(rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("sdi_sf_tmp get temperature high threshold failed with rc %d\n", rc);
                    return rc;
                }
            } else {
                return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
            }
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
    return rc;

}

/*
 * Set the threshold values for a chip refered by resource.
 * This is also a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [in] temperature - threshold value to be set for a given chip.
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_sf_tmp_threshold_set(void *resource_hdl, sdi_threshold_t type, int temperature)
{
    if((temperature > 255) || (temperature < -256))
    {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
}

/*
 * Get the fault status of the chip refered by resource.
 * This is also a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] status - pointer to a buffer to get the status of the chip
 * Return - STD_ERR_OK for success and the respective error code from i2c API in case of failure
 */
static t_std_error sdi_sf_tmp_status_get(void *resource_hdl, bool *status)
{
    sdi_device_hdl_t chip = NULL;
    sdi_sf_tmp_device_t *tmp_data = NULL;
    t_std_error rc = STD_ERR_OK;
    int tmp_status = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp_data = (sdi_sf_tmp_device_t*)chip->private_data;
    STD_ASSERT(tmp_data != NULL);

    if (tmp_data->status_offset_valid == true) {
        rc = sdi_sf_tmp_data_get(resource_hdl, tmp_data->temp_status_addr_offset,
                                 SMF_TMP_REGISTER_ONE_BYTE_WIDTH, &tmp_status);
        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("temperature sensor status get failed with rc %d\n", rc);
            return rc;
        }
        *status = (tmp_status == 0) ? false: true;
    } else {
        rc = SDI_DEVICE_ERRCODE(EOPNOTSUPP);
        SDI_DEVICE_ERRMSG_LOG("temperature sensor status get unsupported, failed with rc %d\n", rc);
    }

    return rc;
}

temperature_sensor_t sdi_sf_tmp_sensor={
        NULL, /*As the init is done as part of chip init, resource init is not required*/
        sdi_sf_tmp_temperature_get,
        sdi_sf_tmp_threshold_get,
        sdi_sf_tmp_threshold_set,
        sdi_sf_tmp_status_get
};

/* Export the Driver table */
sdi_driver_t sdi_sf_tmp_entry={
        sdi_sf_tmp_register,
        sdi_sf_tmp_chip_init
};


/*
 * The config file format will be as below for sdi_sf_tmp devices
 *
 * <sdi_sf_tmp instance="<chip_instance>"
 * addr="<Address of the device>"
 * low_threshold="<low threshold value>" high_threshold="<high threshold value>"
 * alias="<Alias name for the particular devide>">
 * </sdi_sf_tmp>
 * Mandatory attributes    : instance and addr
 */
 /*
 * This the call back function for the device registration
 * [in] node - Config node for the device
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - pointer to the device handle which will be updated by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table fp requires it
 */
t_std_error sdi_sf_tmp_register(std_config_node_t node,void *bus_handle,sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    sdi_sf_tmp_device_t *tmp_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_SF_IO_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);

    tmp_data = calloc(sizeof(sdi_sf_tmp_device_t),1);
    STD_ASSERT(tmp_data != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    chip->callbacks = &sdi_sf_tmp_entry;
    chip->private_data = (void*)tmp_data;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL)
    {
        snprintf(chip->alias,SDI_MAX_NAME_LEN,"sdi_sf_tmp-%d", chip->instance );
    }
    else
    {
        safestrncpy(chip->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_TMP_SENSOR_VALUE_OFFSET);
    if(node_attr != NULL)
    {
        tmp_data->temp_sensor_addr_offset = (uint16_t) strtol(node_attr, NULL, 0);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_TMP_SENSOR_FAULT_STATUS_OFFSET);
    if(node_attr != NULL)
    {
        tmp_data->status_offset_valid = true;
        tmp_data->temp_status_addr_offset = (uint16_t) strtol(node_attr, NULL, 0);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_TMP_SENSOR_LOW_THRESHOLD_OFFSET);
    if(node_attr != NULL)
    {
        tmp_data->low_threshold_offset_valid = true;
        tmp_data->temp_low_threshold_addr_offset = (uint16_t) strtol(node_attr, NULL, 0);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SF_TMP_SENSOR_HIGH_THRESHOLD_OFFSET);
    if(node_attr != NULL)
    {
        tmp_data->high_threshold_offset_valid = true;
        tmp_data->temp_high_threshold_addr_offset = (uint16_t) strtol(node_attr, NULL, 0);
    }

    sdi_resource_add(SDI_RESOURCE_TEMPERATURE,chip->alias,(void*)chip,
            &sdi_sf_tmp_sensor);

    *device_hdl = chip;

    return STD_ERR_OK;
}


t_std_error sdi_sf_tmp_chip_init(sdi_device_hdl_t device_hdl)
{
    return STD_ERR_OK;
}
