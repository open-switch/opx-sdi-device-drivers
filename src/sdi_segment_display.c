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
 * filename: sdi_segment_display.c
 */

/******************************************************************************
* Implements the driver for segment display LED which controlled by pin group
* bus for displaying patterns on the LED.
******************************************************************************/
#include "sdi_led_internal.h"
#include "sdi_pin_group_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_bus_api.h"
#include "sdi_device_common.h"
#include "sdi_common_attr.h"
#include "sdi_resource_internal.h"

#include "std_assert.h"
#include "std_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))

/* Attribute used to define the pattern to turn OFF the segment display */
#define SDI_SEGMENT_DISPLAY_OFF_PATTERN "off_pattern"

/* Default pattern to turn off the display */
#define SDI_SEGMENT_DISPLAY_OFF_PATTERN_DEFAULT 0xFF

/* Name for the map nodes */
#define SDI_SEGMENT_DISPLAY_MAP_NODE    "display_map"
#define SDI_SEGMENT_DISPLAY_MAP_INPUT   "input"
#define SDI_SEGMENT_DISPLAY_MAP_OUTPUT  "output"

#define SDI_SEGMENT_DISPLAY_MAP_SIZE 256

#define SDI_LED_NUM_OF_DIGIT_SUPPORTED 1

typedef struct _sdi_segment_display_data_t {
    uint_t  off_pattern;
    uint_t  last_written;
    uint_t  char_map[SDI_SEGMENT_DISPLAY_MAP_SIZE];
} sdi_segment_display_data_t;

static t_std_error sdi_segment_display_register (std_config_node_t node, void *bus_handle,
                                                    sdi_device_hdl_t* device_hdl);
/**
 * Sets a single pattern on the display
 */
static t_std_error sdi_segment_display_set_value(sdi_device_hdl_t dev_hdl,
                                                 uint_t value)
{
    sdi_pin_group_bus_hdl_t bus_hdl = NULL;
    t_std_error rc;

    bus_hdl = (sdi_pin_group_bus_hdl_t)dev_hdl->bus_hdl;

    rc = sdi_pin_group_acquire_bus(bus_hdl);
    if (rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_pin_group_write_level(bus_hdl, value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Writing value %u to pin group bus failed with rc : %#x",
                              value, rc);
    }
    sdi_pin_group_release_bus(bus_hdl);

    return rc;
}

/**
 * Get the current pattern from the display
 */
static t_std_error sdi_segment_display_get_value(sdi_device_hdl_t dev_hdl,
                                                 uint_t *value)
{
    sdi_pin_group_bus_hdl_t bus_hdl = NULL;
    t_std_error rc;

    bus_hdl = (sdi_pin_group_bus_hdl_t)dev_hdl->bus_hdl;

    rc = sdi_pin_group_acquire_bus(bus_hdl);
    if (rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_pin_group_read_level(bus_hdl, value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Writing value %u to pin group bus failed with rc : %#x",
                              value, rc);
    }
    sdi_pin_group_release_bus(bus_hdl);

    return rc;
}

/**
 * Gets the specified digit on the digital led
 *
 * resource_hdl[in] - Handle of the resource
 * display_string[out] - Value displayed
 * buf_size[in] - size of allocated buffer
 *
 * return t_std_error
 */
static t_std_error sdi_segment_display_get (void *resource_hdl,
    char *display_string, size_t buf_size)
{
    sdi_device_hdl_t segment_led = NULL;
    sdi_segment_display_data_t *data = NULL;
    uint_t read_value = 0;
    uint_t parsed_value = '?'; // Default for unknown
    uint_t i;
    t_std_error rc;

    STD_ASSERT(resource_hdl != NULL);

    segment_led = (sdi_device_hdl_t) resource_hdl;
    data = segment_led->private_data;

    rc = sdi_segment_display_get_value(segment_led, &read_value);
    if (rc != STD_ERR_OK) {
        return rc;
    }

    /* Scan the map data for a matching pattern */
    for (i = 0; i < SDI_SEGMENT_DISPLAY_MAP_SIZE; i++) {
        if (data->char_map[i] == read_value) {
            parsed_value = i;
            break;
        }
    }

    snprintf(display_string, MIN(buf_size, SDI_LED_NUM_OF_DIGIT_SUPPORTED + 1),
             "%c", parsed_value);

    return rc;
}

/**
 * Sets the specified digit on the digital led
 *
 * resource_hdl[in] - Handle of the resource
 * display_string[in] - Value to be displayed
 *
 * return t_std_error
 */
static t_std_error sdi_segment_display_set (void *resource_hdl, const char *display_string)
{
    sdi_device_hdl_t digit_led = NULL;
    sdi_segment_display_data_t *data;
    uint_t led_set_value;
    t_std_error rc;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(display_string != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    data = digit_led->private_data;

    if (strlen(display_string) != SDI_LED_NUM_OF_DIGIT_SUPPORTED) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    /* We only display the first character of the string */
    led_set_value = data->char_map[(uint_t)display_string[0]];

    rc = sdi_segment_display_set_value(digit_led, led_set_value);
    if (rc == STD_ERR_OK) {
        data->last_written = led_set_value;
    }

    return rc;
}

/**
 * Get the seven segment LED power state
 *
 * resource_hdl[in] - Handle of the resource
 * state[out] - bool pointer to state value
 *
 * return t_std_error
 */
static t_std_error sdi_segment_display_get_state (void *resource_hdl,
    bool *state)
{
    t_std_error rc;
    sdi_device_hdl_t digit_led = NULL;
    sdi_segment_display_data_t *data = NULL;
    uint_t read_value;

    STD_ASSERT(resource_hdl != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    data = digit_led->private_data;

    rc = sdi_segment_display_get_value(digit_led, &read_value);
    if (rc == STD_ERR_OK)
    {
        *state = (read_value != data->off_pattern);
    }
    return rc;
}

/**
 * Turn-on the seven segment LED
 *
 * resource_hdl[in] - Handle of the resource
 *
 * return t_std_error
 */
static t_std_error sdi_segment_display_on (void *resource_hdl)
{
    sdi_device_hdl_t digit_led = NULL;
    sdi_segment_display_data_t *data;

    STD_ASSERT(resource_hdl != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    data = digit_led->private_data;

    return sdi_segment_display_set_value(digit_led, data->last_written);
}

/**
 * Turn-off the digital led
 *
 * resource_hdl[in] - Handle of the resource
 *
 * return t_std_error
 */
static t_std_error sdi_segment_display_off (void *resource_hdl)
{
    sdi_device_hdl_t digit_led = NULL;
    sdi_segment_display_data_t *data;

    STD_ASSERT(resource_hdl != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    data = digit_led->private_data;

    return sdi_segment_display_set_value(digit_led, data->off_pattern);
}

static sdi_digital_display_led_t sdi_seven_segment_led = {
    sdi_segment_display_on,
    sdi_segment_display_off,
    sdi_segment_display_set,
    sdi_segment_display_get,
    sdi_segment_display_get_state
};

/**
 * xml format:
 * <segment_display instance="<instance number>"
 *                  off_pattern="<pattern to turn off the display>"
 *                  alias="<led name>">
 *   <display_map input="<ASCII code for input-1>" output="<bit pattern for output>" />
 *   ...
 *   <display_map input="<ASCII code for input-n>" output="<bit pattern for output>" />
 * </segment_display>
 */

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t * sdi_segment_display_entry_callbacks (void)
{
    /* Export Driver table */
    static const sdi_driver_t sdi_segment_display_entry = {
        sdi_segment_display_register,
        /* Init function will be set to NULL for pin-led. All pins are set to
         * default by pin-bus driver */
        NULL
    };

    return &sdi_segment_display_entry;
}

/**
 * Register function for single digit dispaly led which are driving on pin group
 * bus
 *
 * node[in] - Device node from configuration file
 * bus_handle[in] - Parent bus handle of the device
 * device_hdl[out] - Device handler
 *
 * return std_error_t
 */
static t_std_error sdi_segment_display_register (std_config_node_t node, void *bus_handle,
                                                    sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t dev_hdl = NULL;
    sdi_segment_display_data_t *data = NULL;
    std_config_node_t map_node;
    uint_t i;
    uint_t input_character;
    uint_t output_pattern;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT( ((sdi_bus_t*)bus_handle)->bus_type == SDI_PIN_GROUP_BUS);

    dev_hdl = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    /*
     * Deliberately not calling calloc, since the char_map is initialized
     * with a possibly non-zero pattern later in the initialization sequence
     */
    data = malloc(sizeof(sdi_segment_display_data_t));
    STD_ASSERT(data != NULL);

    dev_hdl->bus_hdl = bus_handle;
    dev_hdl->private_data = data;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL){
        snprintf(dev_hdl->alias, SDI_MAX_NAME_LEN, "seven_segment_led_%u",
                 dev_hdl->instance);
    } else {
        safestrncpy(dev_hdl->alias, node_attr, SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_SEGMENT_DISPLAY_OFF_PATTERN);
    if (node_attr == NULL) {
        data->off_pattern = SDI_SEGMENT_DISPLAY_OFF_PATTERN_DEFAULT;
    } else {
        data->off_pattern = (uint_t)strtoul(node_attr, NULL, 0);
    }

    /* Initialize the map to be all off */
    for (i = 0; i < SDI_SEGMENT_DISPLAY_MAP_SIZE; i++) {
        data->char_map[i] = data->off_pattern;
    }

    /* Get the child node */
    map_node = std_config_get_child(node);

    while (map_node != NULL) {
        if (strcmp(SDI_SEGMENT_DISPLAY_MAP_NODE,
                   std_config_name_get(map_node)) != 0) {
            /* Skip over unrecognized nodes */
            map_node = std_config_next_node(map_node);
            continue;
        }

        node_attr = std_config_attr_get(map_node, SDI_SEGMENT_DISPLAY_MAP_INPUT);
        STD_ASSERT (node_attr != NULL);
        input_character = (uint_t)strtoul(node_attr, NULL, 0);

        node_attr = std_config_attr_get(map_node, SDI_SEGMENT_DISPLAY_MAP_OUTPUT);
        STD_ASSERT (node_attr != NULL);
        output_pattern = (uint_t)strtoul(node_attr, NULL, 0);

        if (input_character < SDI_SEGMENT_DISPLAY_MAP_SIZE) {
            data->char_map[input_character] = output_pattern;
        } else {
            /*
             * Discard the map node for invalid input characters,
             * but continue with the configuration.
             */
            SDI_DEVICE_ERRMSG_LOG("Invalid input character %#x for "
                                  SDI_SEGMENT_DISPLAY_MAP_NODE, input_character);
        }

        map_node = std_config_next_node(map_node);
    }

    dev_hdl->callbacks = sdi_segment_display_entry_callbacks();

    sdi_resource_add(SDI_RESOURCE_DIGIT_DISPLAY_LED, dev_hdl->alias, (void*)dev_hdl, &sdi_seven_segment_led);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}
