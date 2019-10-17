/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright (c) 2019 Isaac Lozano <109lozanoi@gmail.com>
 *
 * This code is released using a dual license strategy: BSD/GPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 *
 */

#ifndef __GVUSB2_H__
#define __GVUSB2_H__

#include <linux/module.h>

#define GVUSB2_NUM_URBS 4
#define GVUSB2_NUM_ISOCH_PACKETS 0x100
#define GVUSB2_MAX_AUDIO_PACKET_SIZE 0x100
#define GVUSB2_MAX_VIDEO_PACKET_SIZE 0xc00
#define GVUSB2_VENDOR_ID  0x04bb
#define GVUSB2_PRODUCT_ID 0x0532

#define gvusb2_dbg(dev, fmt, args...) \
	dev_info(dev, fmt, ## args)

struct gvusb2_dev {
	struct usb_device *udev;
};

int gvusb2_read_reg(struct gvusb2_dev *dev, u16 reg, u8 *value);
int gvusb2_write_reg(struct gvusb2_dev *dev, u16 reg, u8 value);
int gvusb2_set_reg_mask(struct gvusb2_dev *dev, u16 reg, u8 mask, u8 value);
int gvusb2_init(struct gvusb2_dev *dev, struct usb_device *udev);
int gvusb2_free(struct gvusb2_dev *dev);

int gvusb2_snd_reset_adc(struct gvusb2_dev *dev);

#endif
