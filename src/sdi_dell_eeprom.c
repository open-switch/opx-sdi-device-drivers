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
 * filename: sdi_dell_eeprom.c
 */


/******************************************************************************
 *  Read the DELL LEGACY eeprom contents like manufacturing, software info blocks
 *  and related function implementations.
 ******************************************************************************/

#include "sdi_device_common.h"
#include "sdi_entity_info.h"
#include "sdi_eeprom.h"
#include "sdi_dell_eeprom.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_entity_info.h"
#include "std_assert.h"
#include "std_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef struct {
    char name[SDI_MAX_NAME_LEN];
    uint32_t offset;
    uint16_t size;
}sdi_eeprom_info;

/* FAN EEPROM Specific Fields */
static sdi_eeprom_info sdi_dell_legacy_fan_eeprom_info[] = {
     /* Name, Offset, Size */
    { SDI_DELL_LEGACY_EEPROM_PPID, SDI_DELL_LEGACY_EEPROM_PPID_OFFSET,
      SDI_DELL_LEGACY_EEPROM_PPID_SIZE },
    { SDI_DELL_LEGACY_EEPROM_REV, SDI_DELL_LEGACY_EEPROM_REV_OFFSET,
      SDI_DELL_LEGACY_EEPROM_REV_SIZE },
    { SDI_DELL_LEGACY_EEPROM_SERVICE_TAG, SDI_DELL_LEGACY_EEPROM_SERVICE_TAG_OFFSET,
      SDI_DELL_LEGACY_EEPROM_SERVICE_TAG_SIZE },
    { SDI_DELL_LEGACY_EEPROM_PART_NUM, SDI_DELL_LEGACY_EEPROM_PART_NUM_OFFSET,
      SDI_DELL_LEGACY_EEPROM_PART_NUM_SIZE },
    { SDI_DELL_LEGACY_FAN_COUNT, SDI_DELL_LEGACY_FAN_COUNT_OFFSET,
      SDI_DELL_LEGACY_FAN_COUNT_SIZE },
    { SDI_DELL_LEGACY_FAN_TYPE, SDI_DELL_LEGACY_FAN_TYPE_OFFSET,
      SDI_DELL_LEGACY_FAN_TYPE_SIZE }
};

/* PSU EEPROM Specific Fields */
static sdi_eeprom_info sdi_dell_legacy_psu_eeprom_info[] = {
     /* Name, Offset, Size */
    { SDI_DELL_LEGACY_EEPROM_PPID, SDI_DELL_LEGACY_EEPROM_PPID_OFFSET,
      SDI_DELL_LEGACY_EEPROM_PPID_SIZE },
    { SDI_DELL_LEGACY_EEPROM_REV, SDI_DELL_LEGACY_EEPROM_REV_OFFSET,
      SDI_DELL_LEGACY_EEPROM_REV_SIZE },
    { SDI_DELL_LEGACY_EEPROM_SERVICE_TAG, SDI_DELL_LEGACY_EEPROM_SERVICE_TAG_OFFSET,
      SDI_DELL_LEGACY_EEPROM_SERVICE_TAG_SIZE },
    { SDI_DELL_LEGACY_EEPROM_PART_NUM, SDI_DELL_LEGACY_EEPROM_PART_NUM_OFFSET,
      SDI_DELL_LEGACY_EEPROM_PART_NUM_SIZE },
    { SDI_DELL_LEGACY_PSU_TYPE, SDI_DELL_LEGACY_PSU_TYPE_OFFSET,
      SDI_DELL_LEGACY_PSU_TYPE_SIZE }
};

/* Delta PSU EEPROM Specific Fields */
static sdi_eeprom_info sdi_delta_psu_eeprom_info[] = {
     /* Name, Offset, Size */
    {SDI_DELTA_PSU_SERIAL_NUM, SDI_DELTA_PSU_SERIAL_NUM_OFFSET,
     SDI_DELTA_PSU_SERIAL_NUM_SIZE},
    {SDI_DELTA_PSU_PART_NUM, SDI_DELTA_PSU_PART_NUM_OFFSET,
     SDI_DELTA_PSU_PART_NUM_SIZE},
    {SDI_DELTA_PSU_LABEL_REV, SDI_DELTA_PSU_LABEL_REV_OFFSET,
     SDI_DELTA_PSU_LABEL_REV_SIZE},
    {SDI_DELTA_PSU_FAN_AIRFLOW, SDI_DELTA_PSU_FAN_AIRFLOW_OFFSET,
     SDI_DELTA_PSU_FAN_AIRFLOW_SIZE},
    {SDI_DELTA_PSU_POWER_SRC, SDI_DELTA_PSU_POWER_SRC_OFFSET,
     SDI_DELTA_PSU_POWER_SRC_SIZE},
    {SDI_DELTA_PSU_POWER_WATT, SDI_DELTA_PSU_POWER_WATT_OFFSET,
     SDI_DELTA_PSU_POWER_WATT_SIZE},
    {SDI_DELTA_PSU_FAN_NUMBER, SDI_DELTA_PSU_FAN_NUMBER_OFFSET,
     SDI_DELTA_PSU_FAN_NUMBER_SIZE},
    {SDI_DELTA_PSU_FAN_SPEED, SDI_DELTA_PSU_FAN_SPEED_OFFSET,
     SDI_DELTA_PSU_FAN_SPEED_SIZE},
};

static void copy_bytes_to_string(char *dst, size_t dst_size, const char *src, size_t src_size)
{
    size_t n = dst_size - 1;
    if (src_size < n)  n = src_size;
    char *d;
    for (d = dst; n > 0; --n, ++d, ++src) {
        char c = *src;
        if (c == 0)  break;
        if (!isprint(c)) {
            *dst = 0;
            return;
        }
        *d = c;
    }
    *d = 0;
}

#define COPY_BYTES_TO_STRING(dst, src, src_size)  (copy_bytes_to_string((dst), sizeof(dst), (src), (src_size)))

static int _atoi(const uint8_t *buf, unsigned size)
{
    char temp[size + 1];
    memcpy(temp, buf, size);
    temp[size] = 0;
    return (atoi(temp));
}

/**
 * Fill Delta PSU EEPROM info into the corresponding entity_info members
 *
 * param[in] parser_type   - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_delta_psu_eeprom_info_fill(sdi_device_hdl_t chip,
        sdi_entity_parser_t parser_type,sdi_entity_info_t *entity_data)
{
    uint8_t offset_index = 0;
    uint8_t total_offsets = 0;
    uint8_t data[SDI_MAX_NAME_LEN];
    uint16_t size = 0;
    t_std_error rc = STD_ERR_OK;
    sdi_eeprom_info *sdi_delta_eeprom_info = NULL;
    entity_info_device_t *eeprom_data = NULL;

    memset(entity_data, 0, sizeof(*entity_data));

    eeprom_data = (entity_info_device_t *)chip->private_data;

    /*
     * Dell EEPROM format does not support no.of fans and maximum speed of the
     * fan. But Entity-Info expect it.Hence return the values that were obtained from
     * configuration file
     */
    entity_data->num_fans = eeprom_data->no_of_fans;
    entity_data->max_speed = eeprom_data->max_fan_speed;

    memset(data, 0, sizeof(data));
    if (parser_type == SDI_DELTA_PSU_EEPROM) {
        size = sizeof(sdi_delta_psu_eeprom_info);
        sdi_delta_eeprom_info = &sdi_delta_psu_eeprom_info[0];
        total_offsets = (size / sizeof(sdi_delta_psu_eeprom_info[0]));
    } else {
        return (SDI_DEVICE_ERR_PARAM);
    }

    while (offset_index < total_offsets) {
        size = sdi_delta_eeprom_info->size;
        rc = sdi_smbus_read_multi_byte(chip->bus_hdl, chip->addr.i2c_addr,
                sdi_delta_eeprom_info->offset,
                data, size, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("SMBUS Read failed:  %d\n", rc);
            return rc;
        }

        if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_SERIAL_NUM_OFFSET){
            COPY_BYTES_TO_STRING(entity_data->ppid, (const char *) data, (size_t) size);
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_PART_NUM_OFFSET){
            COPY_BYTES_TO_STRING(entity_data->part_number, (const char *) data, (size_t) size);
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_LABEL_REV_OFFSET){
            COPY_BYTES_TO_STRING(entity_data->hw_revision, (const char *) data, (size_t) size);
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_POWER_SRC_OFFSET){
            char temp[size + 1];
            memcpy(temp, data, size);
            temp[size] = 0;
            if(strstr(temp, "DC") !=NULL){
                entity_data->power_type.dc_power = true;
            }
            if(strstr(temp, "AC") !=NULL){
                entity_data->power_type.ac_power = true;
            }
            if (!(entity_data->power_type.dc_power || entity_data->power_type.ac_power)) {
                rc = SDI_DEVICE_ERR_CFG;
            }
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_FAN_AIRFLOW_OFFSET){
            if(strncmp((const char *)(data),"REVERSE",7) ==0){
                entity_data->air_flow = SDI_PWR_AIR_FLOW_REVERSE;
            } else if(strncmp((const char *)(data),"FORWARD",7) ==0){
                entity_data->air_flow = SDI_PWR_AIR_FLOW_NORMAL;
            } else {
                rc = SDI_DEVICE_ERR_CFG;
            }
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_POWER_WATT_OFFSET){
            entity_data->power_rating = _atoi(data, size);
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_FAN_NUMBER_OFFSET){
            entity_data->num_fans = _atoi(data, size);
        }else if(sdi_delta_eeprom_info->offset == SDI_DELTA_PSU_FAN_SPEED_OFFSET){
            entity_data->max_speed = _atoi(data, size);
        }

        sdi_delta_eeprom_info++;
        offset_index++;
    }

    return rc;
}

/**
 * Fill manufacturing and software info into the corresponding entity_info members
 *
 * param[in] parser_type   - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_dell_legacy_info_fill(sdi_device_hdl_t chip,
                                             sdi_entity_parser_t parser_type,
                                             sdi_entity_info_t *entity_data)
{
    uint8_t offset_index = 0;
    uint8_t total_offsets = 0;
    uint8_t data[SDI_MAX_NAME_LEN];
    uint16_t size = 0;
    t_std_error rc = STD_ERR_OK;
    sdi_eeprom_info *sdi_dell_legacy_eeprom_info = NULL;
    entity_info_device_t *eeprom_data = NULL;

    memset(entity_data, 0, sizeof(*entity_data));

    eeprom_data = (entity_info_device_t *)chip->private_data;

    /*
     * Dell EEPROM format does not support no.of fans and maximum speed of the
     * fan. But Entity-Info expect it.Hence return the values that were obtained from
     * configuration file
     */
    entity_data->num_fans = eeprom_data->no_of_fans;
    entity_data->max_speed = eeprom_data->max_fan_speed;

    memset(data, 0, sizeof(data));
    switch (parser_type) {
    case SDI_DELL_LEGACY_PSU_EEPROM:
        size = sizeof(sdi_dell_legacy_psu_eeprom_info);
        sdi_dell_legacy_eeprom_info = &sdi_dell_legacy_psu_eeprom_info[0];
        total_offsets = (size / sizeof(sdi_dell_legacy_psu_eeprom_info[0]));
        break;
    case SDI_DELL_LEGACY_FAN_EEPROM:
        size = sizeof(sdi_dell_legacy_fan_eeprom_info);
        sdi_dell_legacy_eeprom_info = &sdi_dell_legacy_fan_eeprom_info[0];
        total_offsets = (size / sizeof(sdi_dell_legacy_fan_eeprom_info[0]));
        break;
    default:
        return (SDI_DEVICE_ERR_PARAM);
    }
    while (offset_index < total_offsets) {
           size = sdi_dell_legacy_eeprom_info->size;
           rc = sdi_smbus_read_multi_byte(chip->bus_hdl, chip->addr.i2c_addr,
                                          sdi_dell_legacy_eeprom_info->offset,
                                          data, size, SDI_I2C_FLAG_NONE);
           if (rc != STD_ERR_OK) {
               SDI_DEVICE_ERRMSG_LOG("SMBUS Read failed:  %d\n", rc);
               return rc;
           }
           if(strncmp(sdi_dell_legacy_eeprom_info->name, SDI_DELL_LEGACY_FAN_TYPE,
                                   strlen(SDI_DELL_LEGACY_FAN_TYPE)) == 0) {
               switch (*data) {
               case SDI_DELL_LEGACY_FAN_AIR_FLOW_NORMAL:
                   entity_data->air_flow = SDI_PWR_AIR_FLOW_NORMAL;
                   break;
               case SDI_DELL_LEGACY_FAN_AIR_FLOW_REVERSE:
                   entity_data->air_flow = SDI_PWR_AIR_FLOW_REVERSE;
                   break;
               default:
                   rc = SDI_DEVICE_ERR_CFG;
              }
           } else if(strncmp(sdi_dell_legacy_eeprom_info->name,
                             SDI_DELL_LEGACY_PSU_TYPE,
                             strlen(SDI_DELL_LEGACY_PSU_TYPE)) == 0) {
               switch (*data) {
               case SDI_DELL_LEGACY_PSU_AC_NORMAL:
                   entity_data->power_type.ac_power = true;
                   entity_data->air_flow            = SDI_PWR_AIR_FLOW_NORMAL;
                   break;
               case SDI_DELL_LEGACY_PSU_AC_REVERSE:
                   entity_data->power_type.ac_power = true;
                   entity_data->air_flow            = SDI_PWR_AIR_FLOW_REVERSE;
                   break;
               case SDI_DELL_LEGACY_PSU_DC_NORMAL:
                   entity_data->power_type.dc_power = true;
                   entity_data->air_flow            = SDI_PWR_AIR_FLOW_NORMAL;
                   break;
               case SDI_DELL_LEGACY_PSU_DC_REVERSE:
                   entity_data->power_type.dc_power = true;
                   entity_data->air_flow            = SDI_PWR_AIR_FLOW_REVERSE;
                   break;
               default:
                   rc = SDI_DEVICE_ERR_CFG;               
               }
           } else if(strncmp(sdi_dell_legacy_eeprom_info->name,
                             SDI_DELL_LEGACY_FAN_COUNT,
                             strlen(SDI_DELL_LEGACY_FAN_COUNT)) == 0) {
                    entity_data->num_fans = *data;
           } else if(strncmp(sdi_dell_legacy_eeprom_info->name,
                             SDI_DELL_LEGACY_EEPROM_PPID,
                             strlen(SDI_DELL_LEGACY_EEPROM_PPID)) == 0) {
               COPY_BYTES_TO_STRING(entity_data->prod_name, (const char *) data, (size_t) size);
               COPY_BYTES_TO_STRING(entity_data->ppid, (const char *) data, (size_t) size);
           } else if (strcmp(sdi_dell_legacy_eeprom_info->name, SDI_DELL_LEGACY_EEPROM_REV) == 0) {
               COPY_BYTES_TO_STRING(entity_data->hw_revision, (const char *) data, (size_t) size);
           } else if (strcmp(sdi_dell_legacy_eeprom_info->name, SDI_DELL_LEGACY_EEPROM_SERVICE_TAG) == 0) {
               COPY_BYTES_TO_STRING(entity_data->service_tag, (const char *) data, (size_t) size);
           } else if (strcmp(sdi_dell_legacy_eeprom_info->name, SDI_DELL_LEGACY_EEPROM_PART_NUM) == 0) {
               COPY_BYTES_TO_STRING(entity_data->part_number, (const char *) data, (size_t) size);
           }

           sdi_dell_legacy_eeprom_info++;
           offset_index++;
    }

    return rc;
}

/**
 * Fill entity_info members for Delta PSU.
 *
 * param[in] parser_type   - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_delta_psu_eeprom_entity_info_fill(sdi_device_hdl_t chip,
                   sdi_entity_parser_t parser_type, sdi_entity_info_t *entity_data)
{
    return(sdi_delta_psu_eeprom_info_fill(chip, parser_type, entity_data));
}

/**
 * Fill entity_info members DELL PSU.
 *
 * param[in] parser_type   - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_dell_legacy_eeprom_entity_info_fill(sdi_device_hdl_t chip,
                   sdi_entity_parser_t parser_type, sdi_entity_info_t *entity_data)
{
    /* Fill Manufacture and Software info from DELL LEGACY EEPROM */
    return(sdi_dell_legacy_info_fill(chip, parser_type, entity_data));
}

/**
 * Read the DELTA EEPROM data and fill the entity_info.
 *
 * param[in] resource_hdl  - resource handler
 * param[in] format        - parser format
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_delta_eeprom_data_get(void *resource_hdl,
                                                   sdi_entity_parser_t format,
                                                   sdi_entity_info_t *entity_info)
{
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;

    /** Validate arguments */
    chip = (sdi_device_hdl_t)resource_hdl;
    STD_ASSERT(chip != NULL);
    STD_ASSERT(entity_info != NULL);

    switch(format) {
          case SDI_DELTA_PSU_EEPROM:
               rc = sdi_delta_psu_eeprom_entity_info_fill(chip,
                                        SDI_DELTA_PSU_EEPROM, entity_info);
               break;
          default:
               rc = SDI_DEVICE_ERRCODE(EPERM);
               break;
    }
    return rc;
}

/**
 * Read the DELL EEPROM data and fill the entity_info.
 *
 * param[in] resource_hdl  - resource handler
 * param[in] format        - parser format
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_dell_legacy_eeprom_data_get(void *resource_hdl,
                                                   sdi_entity_parser_t format,
                                                   sdi_entity_info_t *entity_info)
{
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;

    /** Validate arguments */
    chip = (sdi_device_hdl_t)resource_hdl;
    STD_ASSERT(chip != NULL);
    STD_ASSERT(entity_info != NULL);

    switch(format) {
          case SDI_DELL_LEGACY_PSU_EEPROM:
               rc = sdi_dell_legacy_eeprom_entity_info_fill(chip,
                                        SDI_DELL_LEGACY_PSU_EEPROM, entity_info);
               break;
          case SDI_DELL_LEGACY_FAN_EEPROM:
               rc = sdi_dell_legacy_eeprom_entity_info_fill(chip,
                                        SDI_DELL_LEGACY_FAN_EEPROM, entity_info);
               break;
          default:
               rc = SDI_DEVICE_ERRCODE(EPERM);
               break;
    }
    return rc;
}
/**
 * Delta PSU EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_delta_psu_eeprom_data_get(void *resource_hdl,
                                                sdi_entity_info_t *entity_info)
{
    return sdi_delta_eeprom_data_get(resource_hdl, SDI_DELTA_PSU_EEPROM,
                                                         entity_info);
}

/**
 * Dell  PSU EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_dell_legacy_psu_eeprom_data_get(void *resource_hdl,
                                                sdi_entity_info_t *entity_info)
{
    return sdi_dell_legacy_eeprom_data_get(resource_hdl, SDI_DELL_LEGACY_PSU_EEPROM,
                                                         entity_info);
}

/**
 * FAN EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_dell_legacy_fan_eeprom_data_get(void *resource_hdl,
                                                sdi_entity_info_t *entity_info)
{
    return sdi_dell_legacy_eeprom_data_get(resource_hdl, SDI_DELL_LEGACY_FAN_EEPROM,
                                                         entity_info);
}
