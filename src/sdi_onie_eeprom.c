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
 * filename: sdi_onie_eeprom.c
 */


/******************************************************************************
 *  Read the ONIE TLV eeprom contents and related function implementations.
 *  Also It reads the eeprom content and validate the CRC.
 ******************************************************************************/

#include "sdi_device_common.h"
#include "sdi_entity_info.h"
#include "sdi_eeprom.h"
#include "sdi_onie_eeprom.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_entity_info.h"
#include "std_assert.h"
#include "std_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>       /* used for ntoh and hton operations */
#include <ctype.h>
#include <zlib.h>


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

/**
 * Validates the checksum. Checksum is crc32 based one.
 *
 * param[in] pbuf - Pointer to data buffer
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_onie_validate_checksum(char *pbuf)
{
    uint_t crc = 0, stored_crc = 0;
    sdi_onie_tlv_field *crc_tlv = NULL;
    sdi_onie_tlv_header *ptlv_hdr = (sdi_onie_tlv_header *)pbuf;
    int crc_offset = SDI_TLV_HEADER_SIZE + ntohs(ptlv_hdr->total_len) -
                     (sizeof(sdi_onie_tlv_field) + SDI_ONIE_CRC_SIZE);

    crc_tlv = (sdi_onie_tlv_field*)&pbuf[crc_offset];

    if ((crc_tlv->tag != SDI_ONIE_CRC32_TAG) || (crc_tlv->length != SDI_ONIE_CRC_SIZE)){
        return SDI_DEVICE_ERRNO;
    }

    crc = crc32(0, (void *)pbuf,
                    sizeof(sdi_onie_tlv_header) + ntohs(ptlv_hdr->total_len) -
                    SDI_ONIE_CRC_SIZE);

    stored_crc = ((crc_tlv->value[0] << 24) | (crc_tlv->value[1] << 16) |
                  (crc_tlv->value[2] <<  8) | crc_tlv->value[3]);

    if(crc == stored_crc) {
        return STD_ERR_OK;
    } else {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }
}

/**
 *  Validates the Tlv header.
 *
 * param[in] ptlv_hdr - pointer to Header info
 *
 * return STD_ERR_OK on success and SDI_DEVICE_ERRCODE(EINVAL) on failure
 */
static t_std_error sdi_onie_validate_header(sdi_onie_tlv_header *ptlv_hdr, int size)
{
    if((strncmp(ptlv_hdr->id_string, SDI_ONIE_HDR_ID_STRING,
                                     sizeof(ptlv_hdr->id_string)) == 0) &&
       (ptlv_hdr->hdr_version == SDI_ONIE_HDR_VERSION) &&
       (ntohs(ptlv_hdr->total_len) <= (size - sizeof(sdi_onie_tlv_header)))) {
        return STD_ERR_OK;
    } else {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }
}

 /**
 * Fill vendor extension details in the entity_info members.
 *
 * param[in] tlv_fld         - i/p eeprom content
 * param[in] parser_type     - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return None
 */
static void sdi_onie_fill_vendor_extn(sdi_onie_tlv_field *tlv_fld,
                                      sdi_entity_parser_t parser_type,
                                      sdi_entity_info_t *entity_data)
{
    size_t count = 0;
    const char *value = NULL;
    std_parsed_string_t handle;
    char delimiter[] = SDI_VENDOR_EXTN_DELIMITER;
    char vendor_extn[SDI_MAX_TLV_VALUE_LEN] = {0};
    static const uint8_t iana_num[] = {0x00, 0x00, 0x02, 0xa2, 0x2d};
    static const char iana_ascii[] = "674-";
    uint_t iana_len = 0;

    if (!(parser_type == SDI_ONIE_FAN_EEPROM ||
          parser_type == SDI_ONIE_PSU_EEPROM
          )
        ) {        
        return;
    };

    memcpy(vendor_extn, tlv_fld->value, tlv_fld->length);

    static const char premature_end[] = "Premature end of ONIE EEPROM vendor extension";
    static const char invalid_fmt[]   = "Invalid format for ONIE EEPROM vendor extension";
    const char *err_mesg = 0;
    
    /* We need to understand two formats for the vendor extension (in regexp lingo):

       674-((PSU)|(FANTRAY))-[0-3]-MAXRPM-[0-9]+-FANS-[0-9]+-FF
       or IANA number<0x000002a2>-((PSU)|(FANTRAY))-[0-3]-MAXRPM-[0-9]+-FANS-[0-9]+-FF

       and

       674-[0-3]-[0-9]+-[0-9]+-FF, where the first number is the type of PSU/fan-tray,
       or IANA number<0x000002a2>-[0-3]-[0-9]+-[0-9]+-FF, where the first number is the type of PSU/fan-tray,
       the second number is the number of fans, and the third number is the max fan RPM.
     */

    if (memcmp(vendor_extn, iana_num, sizeof(iana_num)) == 0) {

        iana_len = sizeof(iana_num);
    } else if (strncmp(vendor_extn, iana_ascii, strlen(iana_ascii)) == 0) {

        iana_len = strlen(iana_ascii);
    } else {
        SDI_DEVICE_ERRMSG_LOG("%s", invalid_fmt);
        return;
    }

    if (!std_parse_string(&handle, &vendor_extn[iana_len], delimiter)) {
        SDI_DEVICE_ERRNO_LOG();
        return;
    }



    do {
        /* Skip "PSU" or "FANTRAY", if present;
           if present, make sure it matches what is expected
        */
        value = std_parse_string_next(handle, &count);
        if (value == 0) {
            err_mesg = premature_end;
            break;
        }
        if (strcmp(value, "FANTRAY") == 0 || strcmp(value, "PSU") == 0) {
            if ((strcmp(value, "FANTRAY") == 0 && parser_type != SDI_ONIE_FAN_EEPROM)
                || (strcmp(value, "PSU") == 0 && parser_type != SDI_ONIE_PSU_EEPROM)
                ) {
                err_mesg = "Entity type and ONIE EEPROM vendor extension mismatch";
                break;
            }

            value = std_parse_string_next(handle, &count);
            if (value == 0) {
                err_mesg = premature_end;
                break;
            }
        }

        /* Get PSU or fan tray type */
        uint_t temp = strtoul(value, 0, 0);
        switch (parser_type) {
        case SDI_ONIE_PSU_EEPROM:
            /** Get PSU air flow and input type */
            
            switch (temp) {
            case SDI_ONIE_VENDOR_EXT_PSU_TYPE_AC_NORMAL:
                entity_data->power_type.ac_power = true;
                entity_data->air_flow            = SDI_PWR_AIR_FLOW_NORMAL;
                break;
            case SDI_ONIE_VENDOR_EXT_PSU_TYPE_AC_REVERSE:
                entity_data->power_type.ac_power = true;
                entity_data->air_flow            = SDI_PWR_AIR_FLOW_REVERSE;
                break;
            case SDI_ONIE_VENDOR_EXT_PSU_TYPE_DC_NORMAL:
                entity_data->power_type.dc_power = true;
                entity_data->air_flow            = SDI_PWR_AIR_FLOW_NORMAL;
                break;
            case SDI_ONIE_VENDOR_EXT_PSU_TYPE_DC_REVERSE:
                entity_data->power_type.dc_power = true;
                entity_data->air_flow            = SDI_PWR_AIR_FLOW_REVERSE;
                break;
            default:
                err_mesg = "Invalid value for PSU type in ONIE EEPROM vendor extension";
            }
            break;
            
        case SDI_ONIE_FAN_EEPROM:
            /* Get fan airflow */
            
            switch (temp) {
            case SDI_ONIE_VENDOR_EXT_FAN_TRAY_TYPE_NORMAL:
                entity_data->air_flow = SDI_PWR_AIR_FLOW_NORMAL;
                break;
            case SDI_ONIE_VENDOR_EXT_FAN_TRAY_TYPE_REVERSE:
                entity_data->air_flow = SDI_PWR_AIR_FLOW_REVERSE;
                break;
            default:
                err_mesg = "Invalid value for fan tray type in ONIE EEPROM vendor extension";
            }
            break;
        default:
            break;
        }
        if (err_mesg != 0)  break;

        /* If "MAXRPM", get max RPM, else get number of fans */
        value = std_parse_string_next(handle, &count);
        if (value == 0) {
            err_mesg = premature_end;
            break;
        }
        if (strcmp(value, "MAXRPM") == 0) {
            value = std_parse_string_next(handle, &count);
            if (value == 0) {
                err_mesg = premature_end;
                break;
            }
            entity_data->max_speed = strtoul(value, 0, 0);
        } else {
            entity_data->num_fans = strtoul(value, 0, 0);
        }

        /* If "FANS", get number of fans, else get max RPM */
        value = std_parse_string_next(handle, &count);
        if (value == 0) {
            err_mesg = premature_end;
            break;
        }
        if (strcmp(value, "FANS") == 0) {
            value = std_parse_string_next(handle, &count);
            if (value == 0) {
                err_mesg = premature_end;
                break;
            }
            entity_data->num_fans = strtoul(value, 0, 0);
        } else {
            entity_data->max_speed = strtoul(value, 0, 0);
        }

        /* Check for terminator */
        value = strrchr(&vendor_extn[sizeof(iana_num)], '-');
        if (value == 0) {
            err_mesg = premature_end;
            break;
        }
        if (strcmp(value, "-FF") != 0)  err_mesg = invalid_fmt;
    } while (0);

    if (err_mesg != 0)  SDI_DEVICE_ERRMSG_LOG("%s", err_mesg);

    std_parse_string_free(handle);
}

/**
 * Fill entity_info members in TLV format
 *
 * param[in] eeprom_buf - i/p eeprom content
 * param[in] parser_type   - parser format
 * param[out] entity_data  - entity_info structure to fill
 *
 * return None
 */
static void sdi_onie_fill_tlv_entity_info(char *eeprom_buf, sdi_entity_parser_t parser_type,
                                          sdi_entity_info_t *entity_data)
{
    int tlv_len = 0;
    char *tlv_eeprom = NULL;
    sdi_onie_tlv_field *tlv_fld = NULL;
    sdi_onie_tlv_header *ptlv_hdr = NULL;

    ptlv_hdr = (sdi_onie_tlv_header *)eeprom_buf;
    tlv_len = (ntohs(ptlv_hdr->total_len)) - SDI_TLV_CRC_SIZE;

    tlv_fld = (sdi_onie_tlv_field *)(eeprom_buf + SDI_TLV_HEADER_SIZE);
    tlv_eeprom = (eeprom_buf + SDI_TLV_HEADER_SIZE);

    /** Initialise entity info with zeros. will be used when particular
     * field is not applicable for some devices.
     * Example: fan speed, number of fans are not applicable for
     * system eeprom. hence those fields will be zero.
     */
    memset(entity_data, 0, sizeof(sdi_entity_info_t));

     while (tlv_len > 0) {
        switch(tlv_fld->tag) {
        case SDI_ONIE_PRODUCT_NAME_TAG:
            COPY_BYTES_TO_STRING(entity_data->prod_name, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;

        case SDI_ONIE_SERIAL_NO_TAG:
            COPY_BYTES_TO_STRING(entity_data->ppid, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;

        case SDI_ONIE_SVC_TAG_TAG:
            COPY_BYTES_TO_STRING(entity_data->service_tag, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;

        case SDI_ONIE_LABEL_VERSION_TAG:
            COPY_BYTES_TO_STRING(entity_data->hw_revision, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;
                 
        case SDI_ONIE_PLATFORM_NAME_TAG:
            COPY_BYTES_TO_STRING(entity_data->platform_name, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;

        case SDI_ONIE_VENDOR_TAG:
            COPY_BYTES_TO_STRING(entity_data->vendor_name, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;

        case SDI_ONIE_MAC_ADDR_TAG:
            if (tlv_fld->length == sizeof(entity_data->base_mac)) {
                memcpy(&(entity_data->base_mac), &(tlv_fld->value), tlv_fld->length);
            }
            break;

        case SDI_ONIE_NO_MACS_TAG:
            /** Number of MAC(mac_size) is 2 bytes of data */
            entity_data->mac_size = (tlv_fld->value[1] |
                                     (tlv_fld->value[0] << SDI_BYTE_LENGTH));
            break;
            
        case SDI_ONIE_VENDOR_EXTN_TAG:
            sdi_onie_fill_vendor_extn(tlv_fld, parser_type, entity_data);
            break;
            
        case SDI_ONIE_PART_NO_TAG:
            COPY_BYTES_TO_STRING(entity_data->part_number, (const char *) tlv_fld->value, (size_t) tlv_fld->length);
            break;
            
        default:
            break;
        }

        tlv_eeprom = (tlv_eeprom + (tlv_fld->length + SDI_TLV_FIELD_SIZE));
        tlv_fld = (sdi_onie_tlv_field *)tlv_eeprom;
        tlv_len = tlv_len - tlv_fld->length - SDI_TL_FIELD_SIZE;
    }

    return;
}

/**
 * Read the EEPROM data and fill the entity_info.
 *
 * param[in] resource_hdl    - resource handler
 * param[in] format          - parser format
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_onie_eeprom_data_get(void *resource_hdl,
                                            sdi_entity_parser_t format,
                                            sdi_entity_info_t *entity_info)
{
    char *data = NULL;
    sdi_device_hdl_t chip = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t offset = SDI_EEPROM_START_OFFSET;
    entity_info_device_t *eeprom_data = NULL;
    uint_t total_tlv_data_size=0;

    /** Validate arguments */
    chip = (sdi_device_hdl_t)resource_hdl;
    STD_ASSERT(chip != NULL);
    STD_ASSERT(entity_info != NULL);

    eeprom_data = (entity_info_device_t *)chip->private_data;
    STD_ASSERT(eeprom_data != NULL);

    /** when entity is not assigned or zero, Assert it */
    if (!eeprom_data->entity_size) {
        STD_ASSERT(false);
    }

    data = calloc(eeprom_data->entity_size, 1);
    if (data == NULL) {
        return SDI_DEVICE_ERRCODE(ENOMEM);
    }

    do {
        rc = sdi_device_read(chip, offset, (uint8_t *)data,
                             sizeof(sdi_onie_tlv_header),
                             SDI_I2C_FLAG_NONE);

        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("SMBUS Read failed:  %d", rc);
            break;
        }

        /** Validate TLV Header */
        rc = sdi_onie_validate_header((sdi_onie_tlv_header *)data,
                                      eeprom_data->entity_size);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("EEPROM Content is not in TLV Format.rc=%d \n", rc);
            break;
        }

        total_tlv_data_size=ntohs(((sdi_onie_tlv_header *)data)->total_len);

        /* Now read the additional data */
        rc = sdi_device_read(chip, offset+sizeof(sdi_onie_tlv_header),
                             ((uint8_t*)data)+sizeof(sdi_onie_tlv_header),
                             total_tlv_data_size,SDI_I2C_FLAG_NONE);
        /** Validate Checksum */
        rc = sdi_onie_validate_checksum(data);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("EEPROM Checksum mismatch.rc=%d \n", rc);
            break;
        }

        switch(format) {
              case SDI_ONIE_SYS_EEPROM:
                   sdi_onie_fill_tlv_entity_info(data, SDI_ONIE_SYS_EEPROM, entity_info);
                   break;
              case SDI_ONIE_PSU_EEPROM:
                   sdi_onie_fill_tlv_entity_info(data, SDI_ONIE_PSU_EEPROM, entity_info);
                   if(entity_info->max_speed == 0) {
                       entity_info->max_speed = eeprom_data->max_fan_speed;
                   }
                   if(entity_info->num_fans == 0) {
                       entity_info->num_fans = eeprom_data->no_of_fans;
                   }
                   break;
              case SDI_ONIE_FAN_EEPROM:
                   sdi_onie_fill_tlv_entity_info(data, SDI_ONIE_FAN_EEPROM, entity_info);
                   if(entity_info->max_speed == 0) {
                       entity_info->max_speed = eeprom_data->max_fan_speed;
                   }
                   if(entity_info->num_fans == 0) {
                       entity_info->num_fans = eeprom_data->no_of_fans;
                   }
                   break;
              default:
                   rc = SDI_DEVICE_ERRCODE(EPERM);
        }
    } while(0);

    free(data);
    return rc;
}

/**
 * System EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_sys_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info)
{
    return sdi_onie_eeprom_data_get(resource_hdl, SDI_ONIE_SYS_EEPROM, entity_info);
}

/**
 * PSU EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_psu_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info)
{
    return sdi_onie_eeprom_data_get(resource_hdl, SDI_ONIE_PSU_EEPROM, entity_info);
}

/**
 * FAN EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_fan_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info)
{
    return sdi_onie_eeprom_data_get(resource_hdl, SDI_ONIE_FAN_EEPROM, entity_info);
}


