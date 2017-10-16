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
 * filename: sdi_comm_dev_driver.c
 * Implements APIs to read/write Comm_Dev I2C bus
 */

#include "sdi_comm_dev.h"
#include "sdi_comm_dev_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_comm_dev_attr.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_bit_ops.h"

#include <stdlib.h>
#include <string.h>
#include <linux/i2c.h>

#define NORTHBOUND_MAILBOX_TIME_THRESHOLD     (3)
#define COMM_DEV_STATUS_CLEAR     0x00
static sdi_i2c_addr_t device_i2c_addr = { .i2c_addr = COMM_DEV_I2C_ADDR, .addr_mode_16bit = 0 };
static t_std_error sdi_comm_dev_driver_register(std_config_node_t node, void *bus_handle, sdi_device_hdl_t *device_hdl);
static t_std_error sdi_comm_dev_driver_init(sdi_device_hdl_t device_hdl);
static t_std_error sdi_comm_dev_status_check_and_clear(sdi_resource_hdl_t resource_hdl);

/*
 * Generic read api for Comm_Dev I2C
 */
t_std_error sdi_comm_dev_recv_byte(sdi_device_hdl_t comm_dev_device, uint8_t i2c_addr, uint16_t reg_offset, uint16_t len, uint8_t *regData) {
    t_std_error rc = STD_ERR_OK;
    unsigned int len_count = 0;

    device_i2c_addr.i2c_addr = i2c_addr;

    rc = sdi_smbus_write_byte(comm_dev_device->bus_hdl, device_i2c_addr, (reg_offset & 0xff), (reg_offset >> 8), SDI_I2C_FLAG_NONE);
    if (STD_ERR_OK == rc) {
        for (len_count = 0; len_count < (len); len_count++) {
            rc = sdi_smbus_recv_byte(comm_dev_device->bus_hdl, device_i2c_addr, (uint8_t *)(regData + len_count), SDI_I2C_FLAG_NONE);
        }
    }

    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset 0x%x and #data 0x%x\n", reg_offset, *regData);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("comm_dev smbus read failed at addr : 0x%x reg : 0x%x for %s rc : %d\n",
                              comm_dev_device->addr, reg_offset, comm_dev_device->alias, rc);
    }

    return rc;
}

/*
 * Generic block write api for Comm_Dev I2C
 */
t_std_error sdi_comm_dev_write_block(sdi_device_hdl_t comm_dev_device, uint8_t i2c_addr, uint16_t reg_offset, uint16_t len, uint8_t *regData) {
    t_std_error rc = STD_ERR_OK;

    SDI_DEVICE_TRACEMSG_LOG("SMBUS write block #offset 0x%x and #data 0x%x\n", reg_offset, *regData);

    device_i2c_addr.i2c_addr = i2c_addr;

    rc = sdi_smbus_write_i2c_block_data(comm_dev_device->bus_hdl, device_i2c_addr, reg_offset, len, regData, SDI_I2C_FLAG_NONE);

    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("comm_dev smbus write block failed at addr : 0x%x reg : 0x%x for %s rc : %d\n",
                              comm_dev_device->addr, reg_offset, comm_dev_device->alias, rc);
    }

    return rc;
}

/*
 * Read data from Comm_Dev I2C bus
 */
t_std_error sdi_comm_dev_read(sdi_resource_hdl_t resource_hdl, uint8_t i2c_addr, uint16_t offset, uint16_t len, uint8_t *data) {
    t_std_error rc = STD_ERR_OK;
    sdi_resource_priv_hdl_t comm_dev_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(data != NULL);

    comm_dev_hdl = (sdi_resource_priv_hdl_t)resource_hdl;

    if (comm_dev_hdl->type != SDI_RESOURCE_COMM_DEV) {
        return (SDI_ERRCODE(EPERM));
    }

    rc = sdi_comm_dev_recv_byte(comm_dev_hdl->callback_hdl, i2c_addr, offset, len, data);

    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset 0x%x and #data 0x%x\n", offset, *data);

    return rc;
}

/*
 * Write data to Comm_Dev I2C bus
 */
t_std_error sdi_comm_dev_write(sdi_resource_hdl_t resource_hdl, uint8_t i2c_addr, uint16_t offset, uint16_t len, uint8_t *data) {
    t_std_error rc = STD_ERR_OK;
    sdi_resource_priv_hdl_t comm_dev_hdl = NULL;
    unsigned int len_count = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(data != NULL);

    SDI_DEVICE_TRACEMSG_LOG("SMBUS write #offset 0x%x len %d and #data 0x%x\n", offset, *data);

    comm_dev_hdl = (sdi_resource_priv_hdl_t)resource_hdl;

    if (comm_dev_hdl->type != SDI_RESOURCE_COMM_DEV) {
        return (SDI_ERRCODE(EPERM));
    }

    if (len != 0) {
        while (len_count < len) {
            if ((len - len_count) > (I2C_SMBUS_BLOCK_MAX / 2)) {
                rc = sdi_comm_dev_write_block(comm_dev_hdl->callback_hdl, i2c_addr, (offset + len_count), (I2C_SMBUS_BLOCK_MAX / 2), (data + len_count));
            } else {
                rc = sdi_comm_dev_write_block(comm_dev_hdl->callback_hdl, i2c_addr, (offset + len_count), (len - len_count), (data + len_count));
            }
            if (STD_ERR_OK != rc) {
                break;
            }
            // I2C_SMBUS_BLOCK_MAX / 2 is the maximum value accepted by lower layers
            len_count += (I2C_SMBUS_BLOCK_MAX / 2);
        }
    }

    return rc;
}

/*
 * Verify mailbox header
 * Verifies whether size and checksum of string read from
 * mailbox match size and checksum informed in the first 4 bytes
 * of mailbox buffer
 */
bool check_mailbox_header(uint8_t *p_string, sdi_mailbox_header_t* p_header) {
    uint16_t     str_checksum = 0,
                 checksum_two_complement = 0;
    unsigned int counter = 0;

    for (counter = 0; counter < p_header->size; counter++) {
        str_checksum += *(p_string + counter);
    }
    str_checksum += (p_header->size & 0xFF);
    str_checksum += (p_header->size >> 8);

    checksum_two_complement = ~str_checksum + 1;

    if (checksum_two_complement != p_header->checksum) {
        SDI_DEVICE_ERRMSG_LOG("mbox header checksum error checksum 0x%x calculated checksum 0x%x \n", p_header->checksum, checksum_two_complement);
        return false;
    }

    SDI_DEVICE_TRACEMSG_LOG("mbox header check #size 0x%x and checksum 0x%x\n", p_header->size, p_header->checksum);

    return true;
}

/*
 * Read mailbox data from Comm_Dev I2C bus
 */
static t_std_error sdi_read_mailbox(sdi_resource_hdl_t resource_hdl, uint8_t *regData) {
    t_std_error          rc = STD_ERR_OK;
    sdi_mailbox_header_t mailbox_header;

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_SOUTHBOUND_MAILBOX_START_ADDRESS, SDI_COMM_DEV_MAILBOX_HEADER_SIZE, (uint8_t*) &mailbox_header);

    if (mailbox_header.size > SDI_COMM_DEV_BUFFER_MAX_SIZE) {
        SDI_DEVICE_ERRMSG_LOG("mbox header invalid\n");
        return (SDI_ERRCODE(EPERM));
    }

    if (STD_ERR_OK == rc) {
        rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, (SDI_COMM_DEV_SOUTHBOUND_MAILBOX_START_ADDRESS + SDI_COMM_DEV_MAILBOX_HEADER_SIZE), mailbox_header.size, regData);
    }

    if (STD_ERR_OK == rc) {
        if (!check_mailbox_header(regData, &mailbox_header)) {
            SDI_DEVICE_ERRMSG_LOG("mbox header invalid\n");
            return (SDI_ERRCODE(EPERM));
        }
    }

    return rc;
}

/*
 * Adds size and checksum in the first 4 bytes of mailbox
 */
void add_mailbox_header(uint8_t *p_string, sdi_mailbox_header_t* p_header) {
    uint16_t     str_checksum = 0;
    unsigned int counter = 0;

    // size in header is string length + '\0', it does not include header itself
    p_header->size = strlen((char*) p_string) + 1;

    for (counter = 0; counter < p_header->size; counter++) {
        str_checksum += *(p_string + counter);
    }
    str_checksum += (p_header->size & 0xFF);
    str_checksum += (p_header->size >> 8);

    p_header->checksum = ~str_checksum + 1;

    SDI_DEVICE_TRACEMSG_LOG("mbox header add #size 0x%x and checksum 0x%x\n", p_header->size, p_header->checksum);
}

/*
 * Write mailbox data to Comm_Dev I2C bus
 */
static t_std_error sdi_write_mailbox(sdi_resource_hdl_t resource_hdl, uint16_t len, uint8_t *regData) {
    sdi_resource_priv_hdl_t comm_dev_hdl = NULL;
    t_std_error             rc = STD_ERR_OK;
    unsigned int            len_count = 0;
    sdi_mailbox_header_t    mailbox_header;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(regData != NULL);

    comm_dev_hdl = (sdi_resource_priv_hdl_t)resource_hdl;

    if (comm_dev_hdl->type != SDI_RESOURCE_COMM_DEV) {
        SDI_DEVICE_ERRMSG_LOG("Resource Handler is not COMM DEV\n");
        return (SDI_ERRCODE(EPERM));
    }
    // Check status register and clear it
    // Check status register and clear NB.

    rc = sdi_comm_dev_status_check_and_clear(comm_dev_hdl);

    add_mailbox_header(regData, &mailbox_header);

    rc = sdi_comm_dev_write_block(comm_dev_hdl->callback_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_NORTHBOUND_MAILBOX_START_ADDRESS,
                                  SDI_COMM_DEV_MAILBOX_HEADER_SIZE, (uint8_t*) &mailbox_header);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error writing Mailbox Header rc=%d\n", rc);
        return (SDI_ERRCODE(EPERM));
    }

    while (len_count < len) {
        // I2C_SMBUS_BLOCK_MAX / 2 is the maximum value accepted by lower layers
        if ((len - len_count) > (I2C_SMBUS_BLOCK_MAX / 2)) {
            rc = sdi_comm_dev_write_block(comm_dev_hdl->callback_hdl, COMM_DEV_I2C_ADDR,
                                          (SDI_COMM_DEV_NORTHBOUND_MAILBOX_START_ADDRESS + SDI_COMM_DEV_MAILBOX_HEADER_SIZE + len_count),
                                          (I2C_SMBUS_BLOCK_MAX / 2), (uint8_t *)(regData + len_count));
        } else {
            rc = sdi_comm_dev_write_block(comm_dev_hdl->callback_hdl, COMM_DEV_I2C_ADDR,
                                          (SDI_COMM_DEV_NORTHBOUND_MAILBOX_START_ADDRESS + SDI_COMM_DEV_MAILBOX_HEADER_SIZE + len_count),
                                          (len - len_count), (uint8_t *)(regData + len_count));
        }
        if (STD_ERR_OK != rc) {
            break;
        }
        len_count += (I2C_SMBUS_BLOCK_MAX / 2);
    }

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error writing Mailbox Payload rc=%d\n", rc);
    }
    return rc;
}

/*
 * Write bit Read & Verified using Comm_Dev I2C bus
 */
static t_std_error sdi_package_read_and_verified(sdi_resource_hdl_t resource_hdl) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     mbox_control[SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE] = {0};

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                           SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE, &mbox_control[0]);
    if (STD_ERR_OK == rc) {
        STD_BIT_SET(mbox_control[0], SDI_COMM_DEV_BIT_SB_PACKAGE_READ_AND_VERIFIED);
        rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                                SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE, &mbox_control[0]);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error writing Package Read and Verified flag rc=%d\n", rc);
    }
    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading Package Read and Verified flag rc=%d\n", rc);
    }
    return rc;
}

/*
 * Write bit Package Download Complete using Comm_Dev I2C
 */
static t_std_error sdi_package_download_complete(sdi_resource_hdl_t resource_hdl) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     mbox_control[SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE
                             + SDI_COMM_DEV_NORTHBOUND_MAILBOX_TIME_THRESHOLD_SIZE] = {0};
    uint16_t  *tm_ptr = (uint16_t *) (mbox_control + SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE);

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                           SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE, &mbox_control[0]);
    if (STD_ERR_OK == rc) {
        STD_BIT_SET(mbox_control[0], SDI_COMM_DEV_BIT_NB_PACKAGE_DOWNLOAD_COMPLETE);
        *tm_ptr = NORTHBOUND_MAILBOX_TIME_THRESHOLD;
        rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                                sizeof(mbox_control), &mbox_control[0]);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error writing Package Download Complete flag rc=%d\n", rc);
    }
    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading Package Download Complete flag rc=%d\n", rc);
    }
    return rc;
}

/*
 * Read Chassis Service Tag using Comm_Dev I2C bus
 */
t_std_error sdi_comm_dev_read_svc_tag(sdi_resource_hdl_t resource_hdl, uint8_t *regData) {
    t_std_error rc = STD_ERR_OK;

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_CHASSIS_SERVICE_TAG, SDI_COMM_DEV_CHASSIS_SERVICE_TAG_SIZE, regData);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error reading Chassis Service Tag rc=%d\n", rc);
    }
    return rc;
}

/*
 * Read/Write Host System Firmware Revision using
 * Comm_Dev I2C bus
 */
static t_std_error sdi_read_write_iom_fw_revision(sdi_resource_hdl_t resource_hdl, bool writeFlag, uint8_t *regData) {
    t_std_error rc = STD_ERR_OK;

    if (writeFlag) {
        rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_IOM_VENDOR_FW_REV, SDI_COMM_DEV_IOM_VENDOR_FW_REV_SIZE, regData);
    } else {
        rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_IOM_VENDOR_FW_REV, SDI_COMM_DEV_IOM_VENDOR_FW_REV_SIZE, regData);
    }

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error reading/writing IOM FW Revision writeFlag=%d rc=%d\n", writeFlag, rc);
    }
    return rc;
}

/*
 * Write NPU temperature to Comm Dev sensor# 06 using
 * Comm_Dev I2C bus
 */
static t_std_error sdi_write_sensor_06_telemetry(sdi_resource_hdl_t resource_hdl, int input_temperature) {
    t_std_error rc = STD_ERR_OK;
    uint8_t  temp[SDI_COMM_DEV_SENSOR_06_TELEMETRY_SIZE] = {0};
    uint16_t* p_temperature = (uint16_t*) &temp[0];
    // Comm_Dev expects temperatures in tenths of degree Celsius, so multiply input value by 10
    *p_temperature = input_temperature * 10;

    rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_SENSOR_06_TELEMETRY, SDI_COMM_DEV_SENSOR_06_TELEMETRY_SIZE, &temp[0]);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error writing temperature sensor data rc=%d\n", rc);
    }
    return rc;
}

/*
 * Read Comm Dev Firmware Revision using Comm_Dev I2C bus
 */
t_std_error sdi_comm_dev_read_fw_rev(sdi_resource_hdl_t resource_hdl, uint8_t *regData) {
    t_std_error rc = STD_ERR_OK;

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_DELL_MANAGEMENT_FW_REVS, SDI_COMM_DEV_DELL_MANAGEMENT_FW_REVS_SIZE, regData);

    if (STD_ERR_OK != rc) {
        SDI_DEVICE_ERRMSG_LOG("Error reading COMM DEV FW Revision rc=%d\n", rc);
    }
    return rc;
}

/*
 * Read Host System position slot using Comm_Dev I2C bus
 */
t_std_error sdi_comm_dev_read_iom_slot(sdi_resource_hdl_t resource_hdl, unsigned int *slot) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     slot_occupation[SDI_COMM_DEV_IOM_SLOT_OCCUPATION_SIZE] = {0};

    *slot = 0;
    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_IOM_SLOT_OCCUPATION, SDI_COMM_DEV_IOM_SLOT_OCCUPATION_SIZE, &slot_occupation[0]);

    if (rc == STD_ERR_OK) {
        *slot = slot_occupation[0] & 0x002F;
    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading IOM slot rc=%d\n", rc);
    }

    return rc;
}

/*
 * Read Host System position slot using Comm_Dev I2C bus
 */
static t_std_error sdi_read_platform_info(sdi_resource_hdl_t resource_hdl, sdi_platform_info_t *platform_info) {
    t_std_error rc = STD_ERR_OK;

    memset(platform_info, 0, sizeof(sdi_platform_info_t));

    rc = sdi_comm_dev_read_svc_tag(resource_hdl, (uint8_t*) platform_info->service_tag);
    if (rc == STD_ERR_OK) {
        rc = sdi_comm_dev_read_fw_rev(resource_hdl, (uint8_t*) platform_info->comm_dev_fw_ver);
        if (rc == STD_ERR_OK) {
            rc = sdi_comm_dev_read_iom_slot(resource_hdl, &(platform_info->slot_occupation));
        }
    }

    return rc;
}
/*
 * Flush NorthBound Mailbox using Comm_Dev I2C bus
 */
static t_std_error sdi_flush_msg_buffer(sdi_resource_hdl_t resource_hdl) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     mbox_control[SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE] = {0};

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                           SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE, &mbox_control[0]);
    if (STD_ERR_OK == rc) {
        STD_BIT_SET(mbox_control[0], SDI_COMM_DEV_BIT_NB_FLUSH_MAILBOX);
        rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL,
                                SDI_COMM_DEV_VENDOR_INTELLIGENCE_MAILBOX_CONTROL_SIZE, &mbox_control[0]);
        if (STD_ERR_OK != rc) {
            SDI_DEVICE_ERRMSG_LOG("Error writing Flush Northbound Mailbox flag rc=%d\n", rc);
        }
    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading Flush Northbound Mailbox flag rc=%d\n", rc);
    }

    return rc;
}

/*
 * Read Northbound Mailbox buffer ready/not ready using Comm_Dev I2C bus
 */
static t_std_error sdi_is_buffer_ready(sdi_resource_hdl_t resource_hdl, bool *ready) {
    t_std_error rc = STD_ERR_OK;
    uint8_t     mbox_status[SDI_COMM_DEV_NORTHBOUND_MAILBOX_STATUS_SIZE] = {0};

    *ready = false;
    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_NORTHBOUND_MAILBOX_STATUS, SDI_COMM_DEV_NORTHBOUND_MAILBOX_STATUS_SIZE, &mbox_status[0]);

    if (rc == STD_ERR_OK) {
        if (STD_BIT_TEST(mbox_status[0], SDI_COMM_DEV_BIT_NB_PACKAGE_IN_MAILBOX) == 0) {
            *ready = true;
        }
    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading Northbound Mailbox Status register rc=%d\n", rc);
    }

    return rc;
}

/*
 * Read Comm dev status Alerts and clear
 */

static t_std_error sdi_comm_dev_status_check_and_clear(sdi_resource_hdl_t resource_hdl)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t     comm_dev_status[SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER_SIZE] = {0};

    rc = sdi_comm_dev_read(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER, SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER_SIZE, &comm_dev_status[0]);

    if( rc == STD_ERR_OK) {
        if (STD_BIT_TEST(comm_dev_status[0], SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_RESET) == 0) {
            SDI_DEVICE_TRACEMSG_LOG("Comm dev  reports reset Alarm\n");
        }

        if (STD_BIT_TEST(comm_dev_status[0], SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_PORT_NOTIFICATION) == 0) {
            SDI_DEVICE_TRACEMSG_LOG("Comm dev  reports Downlink port notification Alarm\n");
        }

        if (STD_BIT_TEST(comm_dev_status[0], SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_BIT_NB_TMIEOUT_ALERT) == 0) {
            SDI_DEVICE_TRACEMSG_LOG("Comm dev reports NB timeout Alarm\n");
        }

        uint16_t* p_comm_dev_status  = (uint16_t*) &comm_dev_status[0];
        *p_comm_dev_status = COMM_DEV_STATUS_CLEAR;

        // Clear Alert in comm dev status register
        rc = sdi_comm_dev_write(resource_hdl, COMM_DEV_I2C_ADDR, SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER,
                SDI_COMM_DEV_VENDOR_INTELLIGENCE_STATUS_REGISTER_SIZE, &comm_dev_status[0]);
        if (STD_ERR_OK != rc) {
            SDI_DEVICE_ERRMSG_LOG("Error writing Flush Commdev status Alerts rc=%d\n", rc);
        }

        // Flush NB mailbox
        rc = sdi_flush_msg_buffer(resource_hdl);
        if (STD_ERR_OK != rc) {
            SDI_DEVICE_ERRMSG_LOG("Error in NB Mailbox msg Flush rc=%d\n", rc);
        }

    } else {
        SDI_DEVICE_ERRMSG_LOG("Error reading Commdev Status rc=%d\n", rc);
    }

    return rc;
}

/* Callback handlers for Comm_Dev */
static comm_dev_ctrl_t comm_dev_ctrl = {
    .read_mbox = sdi_read_mailbox,
    .write_mbox = sdi_write_mailbox,
    .write_pckg_read_verif = sdi_package_read_and_verified,
    .write_pckg_downl_compl = sdi_package_download_complete,
    .read_platform_info = sdi_read_platform_info,
    .access_fw_rev = sdi_read_write_iom_fw_revision,
    .write_temp_sensor = sdi_write_sensor_06_telemetry,
    .flush_msg_buffer = sdi_flush_msg_buffer,
    .get_buffer_ready = sdi_is_buffer_ready
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t* sdi_comm_dev_driver_entry_callbacks(void) {
    /*Export Driver table*/
    static const sdi_driver_t comm_dev_driver_entry = {
        sdi_comm_dev_driver_register,
        sdi_comm_dev_driver_init
    };

    return &comm_dev_driver_entry;
}

/*
 * Register function for Comm_Dev
 * node[in]         - Device node from configuration file
 * bus_handle[in]   - Parent bus handle of the device
 * device_hdl[out]  - Device handle which is filled by this function
 * return           - t_std_error
 */
static t_std_error sdi_comm_dev_driver_register(std_config_node_t node, void *bus_handle,
                                                sdi_device_hdl_t *device_hdl) {
    sdi_device_hdl_t dev_hdl = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t *)bus_handle)->bus_type == SDI_I2C_BUS);

    dev_hdl = calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    dev_hdl->bus_hdl = bus_handle;
    dev_hdl->callbacks = sdi_comm_dev_driver_entry_callbacks();

    sdi_resource_add(SDI_RESOURCE_COMM_DEV, "comm_dev", (void *)dev_hdl, &comm_dev_ctrl);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}

/*
 * Initialize the device
 * device_hdl[in] - Handle to the device
 * return         - t_std_error
 */
static t_std_error sdi_comm_dev_driver_init(sdi_device_hdl_t dev_hdl) {
    return STD_ERR_OK;
}




