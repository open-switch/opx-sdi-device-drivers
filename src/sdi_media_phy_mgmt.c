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
 * filename: sdi_media_phy_mgmt.c
 */


/******************************************************************************
 * sdi_media_phy_mgmt.c
 * Implements the copper SFP phy config setting related APIs
 *****************************************************************************/
#include "sdi_sfp_reg.h"
#include "sdi_media.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_time_tools.h"

/* Magic number description not given in appnote from Marvell
   an-2036 app note from Mavell give below magic value to enable
   SGMII mode for phy device */
#define PHY_SGMII_MODE 0x9084

#define EXT_ADD_SERDES_TX_RX_CONTROL        (0x001F)
#define EXT_PHY_CTRL_SERDES_TX_RX_ENABLE    (0x0000)
#define EXT_PHY_CTRL_SERDES_TX_RX_DISABLE   (0x2001)

#define PHY_CTRL_DELAY (100 * 1000)
static sdi_i2c_addr_t sfp_phy_i2c_addr = { .i2c_addr = SFP_PHY_I2C_ADDR, .addr_mode_16bit = 0};

/**
 * Raw write api for media copper sfp
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to write
 * data[in]         - input buffer which contains the data to be written
 * return           - t_std_error
 */

t_std_error sdi_sfp_phy_write(sdi_device_hdl_t sfp_device,int reg_offset,uint16_t reg_data)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t val = 0;

    val = ((reg_data >> 8) | (reg_data << 8));

    SDI_DEVICE_TRACEMSG_LOG("SMBUS Write  #offset %x and #data %x\n", reg_offset, reg_data);
    rc = sdi_smbus_write_word(sfp_device->bus_hdl,sfp_phy_i2c_addr,reg_offset,
            val, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus write failed at addr : %x reg : %x for %s rc : %d",
                sfp_device->addr, reg_offset, sfp_device->alias, rc);
    }
    return rc;
}

/**
 * Raw read api for media eeprom
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to read
 * data[out]      - Data will be filled after read
 */

t_std_error sdi_sfp_phy_read(sdi_device_hdl_t sfp_device,int reg_offset, uint16_t *reg_data)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t word_buf[2] = { 0 };

    rc = sdi_smbus_read_word(sfp_device->bus_hdl,sfp_phy_i2c_addr,reg_offset,
            (uint16_t *)word_buf, SDI_I2C_FLAG_NONE);

    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset %x and #data %x\n", reg_offset, *reg_data);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %x reg : %x for %s rc : %d",
                SFP_PHY_I2C_ADDR, reg_offset, sfp_device->alias, rc);
        return rc;
    }

    *reg_data = ( (word_buf[0] << 8) | (word_buf[1]) );
    return rc;
}

/**
 * media PHY reset in SFP
 * resource_hdl[in] - handle of the resource
 * return           - t_std_error
 */

t_std_error sdi_media_phy_sw_reset(sdi_device_hdl_t sfp_device)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;

    sdi_sfp_phy_read(sfp_device,SFP_COPPER_CTRL_REG,&regData);
    regData |= SFP_COPPER_CTRL_RESET;

    rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_CTRL_REG,regData);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus Write failue failed at addr : %x reg : %x for %s rc : %d",
                sfp_device->addr, SFP_COPPER_CTRL_REG, sfp_device->alias, rc);
    }

    return rc;
}

/**
 * Disable/Enable Auto neg for media PHY in SFP
 * resource_hdl[in] - handle of the resource
 * enable[in]       - "false" to disable and "true" to enable
 * return           - t_std_error
 */
t_std_error sdi_cusfp_phy_autoneg_set (sdi_device_hdl_t sfp_device, bool enable)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;


    do {

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_CTRL_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        if (enable) {
            regData |= SFP_COPPER_CTRL_AE;
        }
        else {
            regData &= ~SFP_COPPER_CTRL_AE;
        }

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_CTRL_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* if Power Down is set. software reset resets
         * the power down bit resulting in link up on the
         * partner.
         * */
        if (!(regData & SFP_COPPER_CTRL_PD))
            rc = sdi_media_phy_sw_reset(sfp_device);

        if (rc != STD_ERR_OK){
            break;
        }

    }while (0);

    return rc;
}


/**
 * SFP Phy mode set (SGMII/GMII/MII ..)
 * resource_hdl[in] - handle of the resource
 * mode[in]         - mode of the interface
 * return           - t_std_error
 */

t_std_error sdi_cusfp_phy_mode_set (sdi_device_hdl_t sfp_device, sdi_media_mode_t mode)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;


    do {
        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_EXT_STATUS_2_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* Details about this magic are not
         * given on the doc. Using hardcoded value. */
        if(mode == SDI_MEDIA_MODE_SGMII)
            regData = PHY_SGMII_MODE;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_EXT_STATUS_2_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* if Power Down is set. software reset resets
         * the power down bit resulting in link up on the
         * partner.
         * */
        if (!(regData & SFP_COPPER_CTRL_PD))
            rc = sdi_media_phy_sw_reset(sfp_device);

        if (rc != STD_ERR_OK){
            break;
        }

    } while (0);

    return rc;
}

/**
 * SFP Phy speed set (1G/100M/10M)
 * resource_hdl[in] - handle of the resource
 * speed[in]         - speed of the interface
 * return           - t_std_error
 */

t_std_error sdi_cusfp_phy_speed_set(sdi_device_hdl_t sfp_device, sdi_media_speed_t speed)
{

    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;
    uint_t  abilGB = 0, abil = 0;


    do{
        if (speed == SDI_MEDIA_SPEED_1G)
            abilGB |= (SFP_COPPER_GB_CTRL_ADV_1000FD);

        if (speed == SDI_MEDIA_SPEED_100M)
            abil |= (SFP_COPPER_ANA_FD_100 | SFP_COPPER_ANA_FD_10);

        if (speed == SDI_MEDIA_SPEED_10M)
            abil |= (SFP_COPPER_ANA_FD_10);

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_ANA_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        regData  &= ~(SFP_COPPER_ANA_HD_10 |
                SFP_COPPER_ANA_FD_10 |
                SFP_COPPER_ANA_HD_100 |
                SFP_COPPER_ANA_FD_100 );

        regData |= abil;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_ANA_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_GB_CTRL_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        regData &= ~(SFP_COPPER_GB_CTRL_ADV_1000FD |
                SFP_COPPER_GB_CTRL_ADV_1000HD);

        regData |= abilGB;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_GB_CTRL_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

    }while(0);

    return rc;
}

/*
 * Get Copper SFP media PHY link status.
 */

t_std_error sdi_cusfp_phy_link_status_get (sdi_device_hdl_t sfp_device, bool *status)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t reg_data = 0;

    do {

        rc = sdi_sfp_phy_read(sfp_device, SFP_COPPER_EXT_STATUS_1_REG, &reg_data);
        if (rc != STD_ERR_OK) {
            break;
        }

        if (reg_data & SFP_COPPER_EXT_STAT_LA) {
            *status = true;
        } else {
            *status = false;
        }

    } while (0);

    return rc;
}

/*
 * Set powerdown enable/disable on Copper SFP PHY.
 */

t_std_error sdi_cusfp_phy_power_down_enable (sdi_device_hdl_t sfp_device, bool enable)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t reg_data = 0;
    uint16_t st_reg_data = 0;

    do {
        rc = sdi_sfp_phy_read(sfp_device, SFP_COPPER_CTRL_REG, &reg_data);
        if (rc != STD_ERR_OK) {
            break;
        }

        rc = sdi_sfp_phy_read(sfp_device, SFP_COPPER_PHY_CTRL_REG, &st_reg_data);
        if (rc != STD_ERR_OK) {
            break;
        }
        if (st_reg_data & SFP_COPPER_PHY_CTRL_MAC_INT_PD) {
            st_reg_data &= ~(SFP_COPPER_PHY_CTRL_MAC_INT_PD);
            rc = sdi_sfp_phy_write(sfp_device, SFP_COPPER_PHY_CTRL_REG, st_reg_data);
            if (rc != STD_ERR_OK) {
                break;
            }
            sdi_media_phy_sw_reset(sfp_device);
        }

        if (enable) {
            if (reg_data & SFP_COPPER_CTRL_PD) break;
            reg_data |= SFP_COPPER_CTRL_PD;
        } else {
            if (!(reg_data & SFP_COPPER_CTRL_PD)) break;
            reg_data &= ~(SFP_COPPER_CTRL_PD);
        }
        
        rc = sdi_sfp_phy_write(sfp_device, SFP_COPPER_CTRL_REG, reg_data);
        if (rc != STD_ERR_OK) {
            break;
        }
    } while (0);

    return rc;
}

/*
 * Enable/Disable Fiber/Serdes Transmitter and Receiver.
 */

t_std_error sdi_cusfp_phy_serdes_control (sdi_device_hdl_t sfp_device, bool enable)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t reg_data = 0;

    do {
        if (enable == true) {
            reg_data = EXT_PHY_CTRL_SERDES_TX_RX_ENABLE;
        } else {
            reg_data = EXT_PHY_CTRL_SERDES_TX_RX_DISABLE;
        }

        rc = sdi_sfp_phy_write(sfp_device, SFP_COPPER_EXT_ADDRESS_REG,
                EXT_ADD_SERDES_TX_RX_CONTROL);
        if (rc != STD_ERR_OK) {
            break;
        }
        std_usleep(PHY_CTRL_DELAY);
        rc = sdi_sfp_phy_write(sfp_device, SFP_COPPER_EXT_PHY_CTRL_REG, reg_data);
        if (rc != STD_ERR_OK) {
            break;
        }
    } while (0);

    return rc;
}
