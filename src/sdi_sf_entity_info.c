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
 * filename: sdi_sf_entity_info.c
 */


/******************************************************************************
 *  Driver implementation for EEPROM (PSU eeprom, FAN eeprom) accessible through
 *  SmartFusion Mailbox
 ******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_bus_api.h"
#include "sdi_pin.h"
#include "sdi_pin_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_device_common.h"
#include "sdi_common_attr.h"
#include "sdi_eeprom.h"
#include "sdi_sf_entity_info_attr.h"
#include "std_assert.h"
#include "sdi_bus_framework.h"
#include "sdi_entity_info_resource_attr.h"
#include "std_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>

/*
 * Private data of Smart Fusion Entity Info device
 */
typedef struct sdi_sf_entity_info_device
{
    uint_t ppid_start_addr;
    uint_t ppid_end_addr;
    uint_t part_num_start_addr;
    uint_t part_num_end_addr;
    uint_t hw_revision_start_addr;
    uint_t hw_revision_end_addr;
    uint_t service_tag_start_addr;
    uint_t service_tag_end_addr;
    uint_t mfg_id_start_addr;
    uint_t mfg_id_end_addr;
    uint_t mfg_date_start_addr;
    uint_t mfg_date_end_addr;
    uint_t serial_number_start_addr;
    uint_t serial_number_end_addr;
    uint_t country_code_start_addr;
    uint_t country_code_end_addr;
    sdi_pin_bus_hdl_t sf_entity_air_flow_status_hdl;
    sdi_pin_bus_hdl_t sf_entity_current_type_hdl;
    uint_t no_of_fans;
    uint_t max_fan_speed;
    bool ppid_addr_valid;     /* Used in cases where PPID is constructed (hence valid=false) rather than read directly */
} sdi_sf_entity_info_device_t;

static t_std_error sdi_sf_entity_info_register(std_config_node_t node,
                                               void *bus_handle,
                                               sdi_device_hdl_t* device_hdl);
static t_std_error sdi_sf_entity_info_init(sdi_device_hdl_t device_hdl);

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t *sdi_sf_entity_info_entry_callbacks(void)
{
    /* Export Driver table */
    static const sdi_driver_t sdi_sf_entity_info_entry = {
        sdi_sf_entity_info_register,
        sdi_sf_entity_info_init
    };
    return &sdi_sf_entity_info_entry;
};

static void entity_info_populate(sdi_device_hdl_t chip, uint_t start, uint_t end,
                                char *data, uint_t data_len)
{
    sdi_bus_hdl_t bus_hdl = NULL;
    uint8_t buf = 0;
    uint_t offset = 0;
    uint_t len = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(chip != NULL);

    bus_hdl = (sdi_bus_hdl_t) chip->bus_hdl;
    STD_ASSERT(bus_hdl != NULL);

    if (start != end) {
        for (offset = start, len = 0; (offset <= end) && (len < data_len); offset++, len++) {
            rc = sdi_bus_read_byte(bus_hdl, chip->addr, offset, &buf);
            if (rc != STD_ERR_OK) {
                SDI_DEVICE_ERRMSG_LOG("entity info byte read failed at offset %x with err %d",
                                      offset, rc);
                data[len] = '\0';
                return;
            }
            if (!isprint(buf)) {
                data[len] = '\0';
                return;
            }
            data[len] = buf;
        }
        data[len] = '\0';
    }
    return;
}

/* Date code is three character code derived from part fields*/
/* Function to construct date code. Result is in-place*/

static void construct_date_code(char* date_str /*Of the form YYMMDD*/)
{
    /* Conversions are based on Dell spec*/
    static const char alphanumeric[] = {'0','1','2','3','4','5','6','7','8','9',
                          'A','B','C','D','E','F','G','H','I','J','K','L',
                          'M','N','O','P','Q','R','S','T','U','V','X','Y','Z'};

    uint_t tmp = strlen(date_str);
    char tmp_str[tmp + 1];
    safestrncpy(tmp_str, date_str, tmp + 1);
    date_str[3] = '\0';

    /* Get day and convert */
    sscanf(tmp_str + 4, "%u", &tmp);
    date_str[2] = alphanumeric[tmp];
    tmp_str[4] = '\0';

    /* Get month and convert */
    sscanf(tmp_str + 2, "%u", &tmp);
    date_str[1] = alphanumeric[tmp];
    tmp_str[2] = '\0';

    /* Get last digit of year */
    date_str[0] = tmp_str[1];
}
t_std_error sdi_sf_entity_info_data_get(void *resource_hdl,
                                        sdi_entity_info_t *entity_info)
{
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;
    sdi_sf_entity_info_device_t *eeprom_data = NULL;
    sdi_pin_bus_level_t level = SDI_PIN_LEVEL_LOW;

    chip = (sdi_device_hdl_t) resource_hdl;
    STD_ASSERT(chip != NULL);
    STD_ASSERT(entity_info != NULL);

    eeprom_data = (sdi_sf_entity_info_device_t *)chip->private_data;
    STD_ASSERT(eeprom_data != NULL);

    entity_info_populate(chip, eeprom_data->part_num_start_addr,
                         eeprom_data->part_num_end_addr, entity_info->part_number, SDI_PART_NUM_LEN);
    entity_info_populate(chip, eeprom_data->hw_revision_start_addr,
                         eeprom_data->hw_revision_end_addr, entity_info->hw_revision, SDI_HW_REV_LEN);
    entity_info_populate(chip, eeprom_data->service_tag_start_addr,
                         eeprom_data->service_tag_end_addr, entity_info->service_tag, NAME_MAX);
    if (eeprom_data->ppid_addr_valid == true)
    {
        entity_info_populate(chip, eeprom_data->ppid_start_addr,
                         eeprom_data->ppid_end_addr, entity_info->ppid, SDI_PPID_LEN);
    }
    else
    {
        /* PPID needs to be constructed from individual fields*/
        char *ptr = entity_info->ppid;
        const uint_t date_code_len = 3;

        uint_t mfg_id_len = 1 + eeprom_data->mfg_id_end_addr
                               - eeprom_data->mfg_id_start_addr;
        uint_t mfg_date_len = 1 + eeprom_data->mfg_date_end_addr
                               - eeprom_data->mfg_date_start_addr;
        uint_t country_code_len = 1 + eeprom_data->country_code_end_addr
                               - eeprom_data->country_code_start_addr;
        uint_t serial_number_len = 1 + eeprom_data->serial_number_end_addr
                               - eeprom_data->serial_number_start_addr;

        /* PPID format: country code|dell part no|mfg id|date code|serial number|hw rev*/

        entity_info_populate(chip, eeprom_data->country_code_start_addr,
                         eeprom_data->country_code_end_addr, ptr, country_code_len);
        ptr += country_code_len;

        safestrncpy(ptr, entity_info->part_number, strlen(entity_info->part_number)+1);
        ptr += strlen(entity_info->part_number);

        entity_info_populate(chip, eeprom_data->mfg_id_start_addr,
                         eeprom_data->mfg_id_end_addr, ptr, mfg_id_len);
        ptr+=mfg_id_len;

        entity_info_populate(chip, eeprom_data->mfg_date_start_addr,
                         eeprom_data->mfg_date_end_addr, ptr, mfg_date_len);
        /* Function is in-place */
        construct_date_code(ptr);
        ptr += date_code_len;

        entity_info_populate(chip, eeprom_data->serial_number_start_addr,
                         eeprom_data->serial_number_end_addr, ptr, serial_number_len);
    }

    if (eeprom_data->sf_entity_air_flow_status_hdl != NULL) {
        rc = sdi_pin_read_level(eeprom_data->sf_entity_air_flow_status_hdl, &level);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("air flow read failed with rc %d\n", rc);
        }
        if (level == SDI_PIN_LEVEL_HIGH) {
            entity_info->air_flow = SDI_PWR_AIR_FLOW_NORMAL;
        } else {
            entity_info->air_flow = SDI_PWR_AIR_FLOW_REVERSE;
        }
    }

    if (eeprom_data->sf_entity_current_type_hdl != NULL) {
        rc = sdi_pin_read_level(eeprom_data->sf_entity_current_type_hdl, &level);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("current type read failed with rc %d\n", rc);
        }
        if (level == 1) {
            entity_info->power_type.ac_power = 1;
            entity_info->power_type.dc_power = 0;
        } else {
            entity_info->power_type.ac_power = 0;
            entity_info->power_type.dc_power = 1;
        }
    }

    entity_info->num_fans = eeprom_data->no_of_fans;
    entity_info->max_speed = eeprom_data->max_fan_speed;

    return rc;
}

entity_info_t sf_entity_info_callback = {
    NULL,
    sdi_sf_entity_info_data_get
};

/**
 * The config file format will be as below for eeprom devices
 *
 * <sf_entity_info instance="<instance>" alias="<alias>"
 * serial_number_start_addr="<addr>" serial_number_end_addr="<addr>"
 * ppid_start_addr="<addr>" ppid_end_adddr="<addr>"
 * part_number_start_addr="<addr>" part_number_end_addr="<addr>"
 * hardware_revision_start_addr="<addr>" hardware_revision_end_addr="<addr>"
 * air_flow_bus_name="<bus_name>" current_type_bus_name="<bus_name>
 * no_of_fans="<fans>" max_fan_speed="<speed>"/>
 *
 */
static t_std_error sdi_sf_entity_info_register(std_config_node_t node,
                                               void *bus_handle,
                                               sdi_device_hdl_t* device_hdl)
{
    char *attr_value = NULL, ppid_address_count = 0;
    sdi_device_hdl_t chip = NULL;
    sdi_sf_entity_info_device_t *eeprom_data = NULL;

    /** Validate arguments */
    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_SF_IO_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);
    eeprom_data = calloc(sizeof(sdi_sf_entity_info_device_t),1);
    STD_ASSERT(eeprom_data != NULL);

    eeprom_data->ppid_addr_valid = false;
    chip->bus_hdl = bus_handle;

    /* Get all config attributes */
    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(attr_value != NULL);
    chip->instance = strtoul(attr_value, NULL, 0);

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (attr_value == NULL) {
        snprintf(chip->alias, SDI_MAX_NAME_LEN, "sf-entity-info-%d", chip->instance );
    } else {
        safestrncpy(chip->alias, attr_value, sizeof(chip->alias));
    }

    chip->callbacks = sdi_sf_entity_info_entry_callbacks();
    chip->private_data = (void*)eeprom_data;

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_PART_NUMBER_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->part_num_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_PART_NUMBER_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->part_num_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_HW_REVISION_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->hw_revision_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_HW_REVISION_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->hw_revision_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_SERVICE_TAG_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->service_tag_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_SERVICE_TAG_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->service_tag_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_PPID_START_ADDR);
    if (attr_value != NULL) {
        ppid_address_count |= 0x01;
        eeprom_data->ppid_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_PPID_END_ADDR);
    if (attr_value != NULL) {
        ppid_address_count |= 0x02;
        eeprom_data->ppid_end_addr = strtoul(attr_value, NULL, 0);
    }

    switch (ppid_address_count) {
        case 0:
            eeprom_data->ppid_addr_valid = false;
            break;
        case 1:
            SDI_DEVICE_ERRMSG_LOG("PPID end address unset in device smf config file");
            return STD_ERR(BOARD, FAIL, ppid_address_count);
        case 2:
            SDI_DEVICE_ERRMSG_LOG("PPID start address unset in device smf config file");
            return STD_ERR(BOARD, FAIL, ppid_address_count);
        case 3:
        default:
            eeprom_data->ppid_addr_valid = true;
            break;
    }
    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_AIR_FLOW_STATUS);
    if (attr_value != NULL) {
        eeprom_data->sf_entity_air_flow_status_hdl = sdi_get_pin_bus_handle_by_name(attr_value);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_CURRENT_TYPE);
    if (attr_value != NULL) {
        eeprom_data->sf_entity_current_type_hdl = sdi_get_pin_bus_handle_by_name(attr_value);
    }

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_NO_OF_FANS);
    if(attr_value) {
        eeprom_data->no_of_fans = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_MAX_SPEED);
    if(attr_value) {
        eeprom_data->max_fan_speed = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_MFG_ID_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->mfg_id_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_MFG_ID_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->mfg_id_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_MFG_DATE_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->mfg_date_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_MFG_DATE_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->mfg_date_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_SERIAL_NUMBER_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->serial_number_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_SERIAL_NUMBER_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->serial_number_end_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_COUNTRY_CODE_START_ADDR);
    if (attr_value != NULL) {
        eeprom_data->country_code_start_addr = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_SF_ENTITY_COUNTRY_CODE_END_ADDR);
    if (attr_value != NULL) {
        eeprom_data->country_code_end_addr = strtoul(attr_value, NULL, 0);
    }

    sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                     &sf_entity_info_callback);
    *device_hdl = chip;

    return STD_ERR_OK;
}

/**
 * Does the initialization for the eeprom device
 * param[in] device_hdl - device handle of the specific device
 * return: STD_ERR_OK
 */
static t_std_error sdi_sf_entity_info_init(sdi_device_hdl_t device_hdl)
{
    return STD_ERR_OK;
}

