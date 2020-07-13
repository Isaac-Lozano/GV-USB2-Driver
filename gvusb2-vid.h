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

#ifndef __GVUSB2_VID_H__
#define __GVUSB2_VID_H__

#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "gvusb2.h"

#define GVUSB2_INPUT_COMPOSITE 0
#define GVUSB2_INPUT_SVIDEO    1
#define GVUSB2_INPUT_MAX_VAL   1

#define GVUSB2_CID_BASE			(V4L2_CID_USER_BASE | 0xf000)
#define GVUSB2_CID_VERTICAL_START	(GVUSB2_CID_BASE + 0)
#define GVUSB2_CID_HORIZONTAL_START	(GVUSB2_CID_BASE + 1)

struct gvusb2_vb {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	int buf_pos;
	int line_pos;
	int field;
};

struct gvusb2_vid {
	/* core data */
	struct gvusb2_dev gv;

	/* usb data */
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep;
	struct urb *urbs[GVUSB2_NUM_URBS];
	/* keeps track of packet seq */
	unsigned char counter;

	/* i2c data */
	struct i2c_adapter adap;
	struct i2c_client i2c_client;

	/* v4l2 data */
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev *sd_tw9910;
	struct video_device vdev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct mutex v4l2_lock;
	unsigned int input_num;
	unsigned int sequence;
	v4l2_std_id standard;

	/* vb2 data */
	struct vb2_queue vb2q;
	struct mutex vb2q_lock;
	struct list_head buf_list;
	spinlock_t buf_list_lock;
	struct gvusb2_vb *current_buf;
};

/* provided by gvusb2-vid.c */
void gvusb2_release(struct v4l2_device *v4l2_dev);
int gvusb2_vid_submit_urbs(struct gvusb2_vid *dev);
void gvusb2_vid_cancel_urbs(struct gvusb2_vid *dev);

/* provided by gvusb2-v4l2.c */
void get_resolution(struct gvusb2_vid *dev, int *width, int *height);
void gvusb2_vid_clear_queue(struct gvusb2_vid *dev);
int gvusb2_vb2_setup(struct gvusb2_vid *dev);
int gvusb2_v4l2_register(struct gvusb2_vid *dev);
void gvusb2_v4l2_unregister(struct gvusb2_vid *dev);
int gvusb2_video_register(struct gvusb2_vid *dev);

/* provided by gvusb2-i2c.c */
int gvusb2_i2c_register(struct gvusb2_vid *dev);
int gvusb2_i2c_unregister(struct gvusb2_vid *dev);

#endif
