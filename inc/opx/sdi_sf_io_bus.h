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
 * filename: sdi_sf_io_bus.h
 */

#ifndef __SDI_SF_IO_BUS_H__
#define __SDI_SF_IO_BUS_H__

#include "sdi_smartfusion_io.h"

typedef struct sf_io_bus {
    sdi_sf_io_bus_t bus;
    uint_t ram_addr_high_io_addr;
    uint_t ram_addr_low_io_addr;
    uint_t ram_read_data_io_addr;
    uint_t ram_write_data_io_addr;
} sf_io_bus_t;

#endif /* __SDI_SF_IO_BUS_H__ */
