/*
 * mms144.h - Platform data for Melfas MMS144 touch driver
 *
 * Copyright (C) 2011 Google Inc.
 * Author: Dima Zavin <dima@xxxxxxxxxxx>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_MMS144_H
#define __LINUX_MMS144_H

struct mms144_platform_data {
 int max_x;
 int max_y;

 bool invert_x;
 bool invert_y;

 int gpio_sda;
 int gpio_scl;
 int gpio_resetb;
 int gpio_vdd_en;

 int (*mux_fw_flash)(bool to_gpios);
 int fpcb_version;
};
#endif /* __LINUX_MMS144_H */