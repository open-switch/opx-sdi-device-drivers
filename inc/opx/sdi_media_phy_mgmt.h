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
 * filename: sdi_media_phy_mgmt.h
 */


/*******************************************************************
* @file   sdi_media_phy_mgmt.h
* @brief  Declares the externel phy and copper SFP config handling API's.
*******************************************************************/

#ifndef _SDI_MEDIA_PHY_MGMT_H_
#define _SDI_MEDIA_PHY_MGMT_H_

#include "sdi_media.h"
#include "sdi_resource_internal.h"
#include "sdi_pin_group.h"


/**
 * @brief Api to enable/disable Autoneg on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] speed - speed of the interface. Should be of type @ref sdi_media_speed_t .
 * @return - standard @ref t_std_error
 *
 */

t_std_error sdi_cusfp_phy_autoneg_set (sdi_device_hdl_t sfp_device, bool enable);

/**
 * @brief Api to set mode on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] mode - SGMII/MII/GMII, Should be of type Refer @ref sdi_media_mode_t.
 * @return - standard @ref t_std_error
 *
 */

t_std_error sdi_cusfp_phy_mode_set (sdi_device_hdl_t sfp_device, sdi_media_mode_t mode);

/**
 * @brief Api to  set speed  on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] speed - speed of the interface. Should be of type @ref sdi_media_speed_t .
 * @return - standard @ref t_std_error
 *
 */

t_std_error sdi_cusfp_phy_speed_set(sdi_device_hdl_t sfp_device, sdi_media_speed_t speed);

/**
 * @brief Api to get link status from media PHY.
 * @param[in] resource_hdl - handle to media
 * @param[in] status - true when link is up, false when link is down
 * @return - standard @ref t_std_error
 *
 */
t_std_error sdi_cusfp_phy_link_status_get (sdi_device_hdl_t sfp_device, bool *status);

/**
 * @brief Api to power down enable/disable on media PHY.
 * @param[in] resource_hdl - handle to media
 * @param[in] enable - true power down media PHY, false - power up media PHY.
 * @return - standard @ref t_std_error
 *
 */
t_std_error sdi_cusfp_phy_power_down_enable (sdi_device_hdl_t sfp_device, bool enable);

/**
 * @brief Api to enable/disable the Fiber/Serdes TX and RX on media PHY.
 * @param[in] resource_hdl - handle to media
 * @param[in] enable - true enable Serdes, false - Disable Serdes.
 * @return - standard @ref t_std_error
 *
 */
t_std_error sdi_cusfp_phy_serdes_control (sdi_device_hdl_t sfp_device, bool enable);

/*
 * Api's to handle extended phy related operations for QSFP_4X1_1000BASE_T media type.
 */

/*
 * sdi_qsfp_4X1_1000baseT_mode_set is to set the mode on remote copper phy
 * for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_mode_set (sdi_device_hdl_t qsfp_device,
                                             uint_t channel, sdi_media_mode_t mode);
/*
 * sdi_qsfp_4X1_1000baseT_speed_set is to set the supported speed on remote
 * copper phy for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_speed_set (sdi_device_hdl_t qsfp_device,
                                              uint_t channel, sdi_media_speed_t speed);

/*
 * sdi_qsfp_4X1_1000baseT_autoneg_set is to set the autoneg on remote
 * copper phy for specified channel.
 */

t_std_error sdi_qsfp_4X1_1000baseT_autoneg_set (sdi_device_hdl_t qsfp_device,
                                                uint_t channel, bool enable);

#endif

