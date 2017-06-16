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
 * filename: sdi_nvram.c
 */

#include "sdi_max6620.h"
#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_nvram_internal.h"
#include "sdi_common_attr.h"
#include "sdi_nvram_resource_attr.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_device_common.h"
#include "std_assert.h"
#include "std_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


/*
 * NVRAM device private data
 */
typedef struct nvram_device {
    char   *alias; /* Alias name */
    char   *dev;   /* Device filename to use, e.g. raw disk; 0 => Use chassis EEPROM */
    off_t  ofs;    /* Offset (bytes) into above device where NVRAM starts */
    uint_t size;   /* Size of NVRAM (bytes) */
} nvram_device_t;

static t_std_error nvram_size(void *hdl, uint_t *size)
{
    *size = ((nvram_device_t *) ((sdi_device_hdl_t) hdl)->private_data)->size;
    
    return (STD_ERR_OK);
}

static t_std_error nvram_read(void *hdl, uint8_t *buf, uint_t ofs, uint_t len)
{
    sdi_device_hdl_t chip        = (sdi_device_hdl_t) hdl;
    nvram_device_t   *nvram_data = (nvram_device_t *) chip->private_data;
    if ((ofs + len) > nvram_data->size) {
        SDI_DEVICE_ERRMSG_LOG("Attempt to read past end of NVRAM %s", nvram_data->alias);
        
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }
    
    t_std_error rc = STD_ERR_OK;
    if (nvram_data->dev != 0) {
        int fd = open(nvram_data->dev, O_RDONLY);
        if (fd < 0
            || lseek(fd, nvram_data->ofs + ofs, SEEK_SET) < 0
            || read(fd, buf, len) != len
            ) {
            rc = SDI_DEVICE_ERRCODE(errno);
        }
        if (fd >= 0)  close(fd);
    } else {
        rc = sdi_smbus_read_multi_byte(chip->bus_hdl,
                                       chip->addr.i2c_addr,
                                       nvram_data->ofs + ofs,
                                       buf,
                                       len,
                                       SDI_I2C_FLAG_NONE
                                       );
    }
    
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Device read failed for NVRAM %s", nvram_data->alias);
    }
    
    return (rc);
}

static t_std_error nvram_write(void *hdl, uint8_t *buf, uint_t ofs, uint_t len)
{
    sdi_device_hdl_t chip        = (sdi_device_hdl_t) hdl;
    nvram_device_t   *nvram_data = (nvram_device_t *) chip->private_data;
    if ((ofs + len) > nvram_data->size) {
        SDI_DEVICE_ERRMSG_LOG("Attempt to write past end of NVRAM %s", nvram_data->alias);

        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    t_std_error rc = STD_ERR_OK;
    if (nvram_data->dev != 0) {
        int fd = open(nvram_data->dev, O_WRONLY);
        if (fd < 0
            || lseek(fd, nvram_data->ofs + ofs, SEEK_SET) < 0
            || write(fd, buf, len) != len
            ) {
            rc = SDI_DEVICE_ERRCODE(errno);
        }
        if (fd >= 0)  close(fd);
    } else {
        rc = sdi_smbus_write_multi_byte(chip->bus_hdl,
                                        chip->addr.i2c_addr,
                                        nvram_data->ofs + ofs,
                                        buf,
                                        len,
                                        SDI_I2C_FLAG_NONE
                                        );
    }

    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Device write failed for NVRAM %s", nvram_data->alias);
    }
    
    return (rc);
}

static nvram_t nvram_resource_callbacks[1] = {
    { .size  = nvram_size,
      .read  = nvram_read,
      .write = nvram_write
    }
};


static t_std_error sdi_nvram_init(sdi_device_hdl_t device_hdl)
{
    return (STD_ERR_OK);
}

static t_std_error sdi_nvram_register(std_config_node_t node,
                                      void              *bus_handle,
                                      sdi_device_hdl_t  *device_hdl
                                      );

/* Export the Driver table */
const sdi_driver_t nvram_entry[1] = {
    { .register_fn = sdi_nvram_register,
      .init        = sdi_nvram_init
    }
};

static t_std_error sdi_nvram_register(
    std_config_node_t node,
    void              *bus_handle,
    sdi_device_hdl_t  *device_hdl
                               )
{
    sdi_device_hdl_t chip = calloc(sizeof(*chip), 1);
    STD_ASSERT(chip != NULL);

    chip->bus_hdl   = bus_handle;
    chip->callbacks = nvram_entry;

    nvram_device_t *nvram_data = calloc(sizeof(*nvram_data), 1);
    STD_ASSERT(nvram_data != NULL);

    chip->private_data = nvram_data;

    char *node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL) {
        snprintf(nvram_data->alias = malloc(SDI_MAX_NAME_LEN),
                 SDI_MAX_NAME_LEN,
                 "nvram-%d",
                 chip->instance
                 );
    } else {
        nvram_data->alias = strdup(node_attr);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_NVRAM_DEV);
    if (node_attr != 0) {
        nvram_data->dev = strdup(node_attr);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    if (node_attr == 0) {
        STD_ASSERT(nvram_data->dev != 0);
    } else {
        STD_ASSERT(nvram_data->dev == 0);
        chip->addr.i2c_addr.i2c_addr = (i2c_addr_t ) strtoul(node_attr, NULL, 16);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_NVRAM_OFS);
    STD_ASSERT(node_attr != NULL);
    nvram_data->ofs = strtoull(node_attr, NULL, 10);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_NVRAM_SIZE);
    STD_ASSERT(node_attr != NULL);
    nvram_data->size = strtoul(node_attr, NULL, 10);

    sdi_resource_add(SDI_RESOURCE_NVRAM,
                     nvram_data->alias,
                     chip,
                     nvram_resource_callbacks
                     );

    *device_hdl = chip;

    return (STD_ERR_OK);
}
