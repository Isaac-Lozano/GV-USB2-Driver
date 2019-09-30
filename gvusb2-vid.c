#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <media/v4l2-dev.h>

#include "gvusb2-vid.h"

MODULE_DESCRIPTION("gvusb2 video driver");
MODULE_AUTHOR("Isaac Lozano <109lozanoi@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");

static const struct usb_device_id gvusb2_id_table[] = {
    { USB_DEVICE(GVUSB2_VENDOR_ID, GVUSB2_PRODUCT_ID) },
    { }
};
MODULE_DEVICE_TABLE(usb, gvusb2_id_table);

/*****************************************************************************
 * Data Processing Functions
 ****************************************************************************/

static inline
struct gvusb2_vb *gvusb2_vid_next_buffer(struct gvusb2_vid *dev)
{
    struct gvusb2_vb *buf = NULL;
    unsigned long flags;

    WARN_ON(dev->current_buf);

    spin_lock_irqsave(&dev->buf_list_lock, flags);

    if (!list_empty(&dev->buf_list)) {
        buf = list_first_entry(&dev->buf_list, struct gvusb2_vb, list);
        list_del(&buf->list);
    }

    spin_unlock_irqrestore(&dev->buf_list_lock, flags);

    return buf;
}

static inline
void gvusb2_vid_copy_video(struct gvusb2_vid *dev, u8 *buf, int len)
{
    int bytes_per_line, width;
    struct gvusb2_vb *vb = dev->current_buf;

//    gvusb2_dbg(&dev->intf->dev, "adding %d bytes to %ld used\n", len, bytes_used);

    get_resolution(dev, &width, 0);
    bytes_per_line = width * 2;

    while (len > 0) {
        int bytes_left_in_line = bytes_per_line - vb->line_pos;
        int len_to_copy;

        if (len < bytes_left_in_line) {
            len_to_copy = len;
        } else {
            len_to_copy = bytes_left_in_line;
        }

        if (vb->buf_pos + len_to_copy > vb->vb.vb2_buf.planes[0].length) {
            printk(KERN_WARNING "gvusb2-vid: buffer overflow detected.\n");

            /* this seems to break it somehow? */
//            memcpy(vb2_plane_vaddr(&vb->vb.vb2_buf, 0) + vb->buf_pos, buf, vb->vb.vb2_buf.planes[0].length - vb->buf_pos);
            return;
        }

        memcpy(vb2_plane_vaddr(&vb->vb.vb2_buf, 0) + vb->buf_pos, buf, len_to_copy);
        vb->buf_pos += len_to_copy;
        vb->line_pos += len_to_copy;
        len -= len_to_copy;
        buf += len_to_copy;

        if (vb->line_pos == bytes_per_line) {
            vb->buf_pos += bytes_per_line;
            vb->line_pos = 0;
        }
    }
}

static inline
void gvusb2_vid_process_data(struct gvusb2_vid *dev, u8 *buf, int len)
{
    if (len < 4) {
        gvusb2_dbg(&dev->intf->dev, "video packet too small\n");
        return;
    }

    if ((buf[0] & 0x80) == 0) {
        if (dev->counter != buf[1]) {
            gvusb2_dbg(&dev->intf->dev, "counter desync. expected %d got %d\n", dev->counter, buf[1]);
        }
        dev->counter = (buf[1] + 1) % 64;
    } else {
        //gvusb2_dbg(&dev->intf->dev, "got field counter %d\n", buf[1]);
        dev->counter = (dev->counter + 1) % 64;
    }

    if (buf[0] == 0xc0) {
//        gvusb2_dbg(&dev->intf->dev, "got frame start\n");

        if (dev->current_buf != NULL) {
//            gvusb2_dbg(&dev->intf->dev, "submitting buffer\n");

//            while (vb2_get_plane_payload(&dev->current_buf->vb.vb2_buf, 0) < dev->current_buf->vb.planes[0].length) {
//                unsigned long bytes_used = vb2_get_plane_payload(&dev->current_buf->vb.vb2_buf, 0);
//                u8 *p = vb2_plane_vaddr(&dev->current_buf->vb.vb2_buf, 0) + bytes_used;
//
//                *p = 0x80;
//
//                vb2_set_plane_payload(&dev->current_buf->vb.vb2_buf, 0, bytes_used + 1);
//            }
            /* submit buffer */
            vb2_set_plane_payload(&dev->current_buf->vb.vb2_buf, 0, dev->current_buf->vb.vb2_buf.planes[0].length);

            dev->current_buf->vb.sequence = dev->sequence++;
            dev->current_buf->vb.field = V4L2_FIELD_INTERLACED;
            dev->current_buf->vb.vb2_buf.timestamp = ktime_get_ns();
            vb2_buffer_done(&dev->current_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

            dev->current_buf = NULL;
        }

        /* get a new buffer */
        dev->current_buf = gvusb2_vid_next_buffer(dev);
        if (dev->current_buf == NULL) {
            gvusb2_dbg(&dev->intf->dev, "null buffer\n");
            return;
        }
        
    }

    if (dev->current_buf == NULL) {
        return;
    }

    if ((buf[0] & 0x80) != 0) {
        if ((buf[0] & 0x40) == 0) {
            int width;
            get_resolution(dev, &width, 0);
            dev->current_buf->buf_pos = width * 2;
            dev->current_buf->line_pos = 0;
        }
        /* TODO: field stuff */
//        gvusb2_dbg(&dev->intf->dev, "got field_marker\n");
        if (buf[0] == 0xc0 || buf[0] == 0x80) {
            return;
        }
    }

    gvusb2_vid_copy_video(dev, buf + 4, len - 4);
}

static inline
void gvusb2_vid_process_isoc(struct gvusb2_vid *dev, struct urb *urb)
{
    int i;

//    gvusb2_dbg(&dev->intf->dev, "got %d packets\n", urb->number_of_packets);
//    gvusb2_dbg(&dev->intf->dev, "first count is %d\n", ((char *)urb->transfer_buffer)[1]);

    for (i = 0; i < urb->number_of_packets; i++) {
        int status = urb->iso_frame_desc[i].status;
        if (status < 0) {
            gvusb2_dbg(&dev->intf->dev, "iso packet failed, skipping (%d)\n", status);
        } else {
            u8 *data = urb->transfer_buffer + urb->iso_frame_desc[i].offset;
            int len = urb->iso_frame_desc[i].actual_length;

            gvusb2_vid_process_data(dev, data, len);
        }
    }
}

static void gvusb2_vid_isoc_irq(struct urb *urb)
{
    int i, ret;
    struct gvusb2_vid *dev = urb->context;
//    gvusb2_dbg(&dev->intf->dev, "gvusb2_vid_isoc_irq(dev)\n");


    switch (urb->status) {
    case 0:
        break;
    case -ECONNRESET:
    case -ENOENT:
    case -ESHUTDOWN:
        return;
    default:
        gvusb2_dbg(&dev->intf->dev, "urb error! status %d\n", urb->status);
        return;
    }

    gvusb2_vid_process_isoc(dev, urb);

    for (i = 0; i < urb->number_of_packets; i++) {
        urb->iso_frame_desc[i].status = 0;
        urb->iso_frame_desc[i].actual_length = 0;
    }

    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret) {
        gvusb2_dbg(&dev->intf->dev, "urb resubmit failed (%d)\n", ret);
    }
}

/*****************************************************************************
 * USB functions
 ****************************************************************************/

void gvusb2_vid_free_urbs(struct gvusb2_vid *dev)
{
    int i;

    for(i = 0; i < GVUSB2_NUM_URBS; i++) {
        struct urb *urb = dev->urbs[i];
        if (urb != NULL) {
            kfree(urb->transfer_buffer);

            usb_free_urb(urb);
            dev->urbs[i] = NULL;
        }
    }
}

int gvusb2_vid_allocate_urbs(struct gvusb2_vid *dev)
{
    int i;

    for(i = 0; i < GVUSB2_NUM_URBS; i++) {
        struct urb *urb;
        int total_offset, pidx;

        urb = usb_alloc_urb(GVUSB2_NUM_ISOCH_PACKETS, GFP_KERNEL);
        if (urb == NULL) {
            gvusb2_vid_free_urbs(dev);
            return -ENOMEM;
        }
        dev->urbs[i] = urb;

        urb->transfer_buffer = kzalloc(
            GVUSB2_NUM_ISOCH_PACKETS * GVUSB2_MAX_VIDEO_PACKET_SIZE,
            GFP_KERNEL);
        if (urb->transfer_buffer == NULL) {
            gvusb2_vid_free_urbs(dev);
            return -ENOMEM;
        }

        urb->dev = dev->gv.udev;
        urb->pipe = usb_rcvisocpipe(dev->gv.udev, 0x02);
        urb->transfer_buffer_length =
            GVUSB2_NUM_ISOCH_PACKETS * GVUSB2_MAX_VIDEO_PACKET_SIZE;
        urb->complete = gvusb2_vid_isoc_irq;
        urb->context = dev;
        urb->interval = 1;
        urb->start_frame = 0;
        urb->number_of_packets = GVUSB2_NUM_ISOCH_PACKETS;
        urb->transfer_flags = URB_ISO_ASAP;

        total_offset = 0;
        for (pidx = 0; pidx < GVUSB2_NUM_ISOCH_PACKETS; pidx++) {
            urb->iso_frame_desc[pidx].offset = total_offset;
            urb->iso_frame_desc[pidx].length =
                GVUSB2_MAX_VIDEO_PACKET_SIZE;
            total_offset += GVUSB2_MAX_VIDEO_PACKET_SIZE;
        }
    }

    return 0;
}

int gvusb2_vid_submit_urbs(struct gvusb2_vid *dev)
{
    int i;
    int ret;

    for(i = 0; i < GVUSB2_NUM_URBS; i++) {
        ret = usb_submit_urb(dev->urbs[i], GFP_KERNEL);
        if (ret < 0) {
            gvusb2_dbg(&dev->intf->dev, "urb submit failed (%d)\n", ret);
            return ret;
        }
    }

    return 0;
}

void gvusb2_vid_cancel_urbs(struct gvusb2_vid *dev)
{
    int i;

    for (i = 0; i < GVUSB2_NUM_URBS; i++) {
        usb_kill_urb(dev->urbs[i]);
    }
}

/*****************************************************************************
 * STK1150 control functions
 ****************************************************************************/

struct regval {
    u16 reg;
    u8 val;
};

static void gvusb2_stk1150_reset_gpio(struct gvusb2_vid *dev)
{
    /* set video decoder pin to output */
    gvusb2_set_reg_mask(&dev->gv, 0x0002, 0x08, 0x08);
    /* set video decoder pin to high */
    gvusb2_set_reg_mask(&dev->gv, 0x0000, 0x08, 0x08);
}

static void gvusb2_stk1150_init_inner(struct gvusb2_vid *dev)
{
    int i;

    static const struct regval phase3[] = {
        {0x000d, 0x00},
        {0x000f, 0x00},
        {0x0018, 0x10},
        {0x001b, 0x0e},
        {0x001c, 0x46},
        {0x001a, 0x14},
        {0x0019, 0x00},
        {0x0300, 0x12},
        {0x0350, 0x41},
        {0x0351, 0x00},
        {0x0352, 0x00},
        {0x0353, 0x00},
        {0x0300, 0x80},
        {0x0018, 0x10},
        {0x0103, 0x00},
        {0x0110, 0x03},
        {0x0111, 0x00},
        {0x0112, 0x02},
        {0x0113, 0x00},
        {0x0114, 0xa3},
        {0x0115, 0x05},
        {0x0116, 0xf2},
        {0x0117, 0x00},
        {0x0100, 0x33},
        {0x0202, 0x08},

        {0xffff, 0xff}
    };

    for (i = 0; phase3[i].reg != 0xffff; i++) {
        gvusb2_write_reg(&dev->gv, phase3[i].reg, phase3[i].val);
    }
}

static void gvusb2_stk1150_reset_slave_ics(struct gvusb2_vid *dev)
{
    u8 reg_0000;

    gvusb2_read_reg(&dev->gv, 0x0000, &reg_0000);

    /* toggle video decoder slave reset pins via GPIO */
    gvusb2_write_reg(&dev->gv, 0x0000, reg_0000 & 0xf7);
    usleep_range(10 * USEC_PER_MSEC, 20 * USEC_PER_MSEC);
    gvusb2_write_reg(&dev->gv, 0x0000, (reg_0000 & 0xf7) | 0x08);
    usleep_range(10 * USEC_PER_MSEC, 20 * USEC_PER_MSEC);
}

static void gvusb2_stk1150_init(struct gvusb2_vid *dev)
{
    gvusb2_stk1150_reset_gpio(dev);
    gvusb2_stk1150_reset_slave_ics(dev);
    gvusb2_stk1150_init_inner(dev);
}

/*****************************************************************************
 * Driver functions
 ****************************************************************************/

int gvusb2_vid_free(struct gvusb2_vid *dev)
{
    gvusb2_dbg(&dev->intf->dev, "gvusb2_vid_free(%p)\n", dev);

    /* free urbs */
    gvusb2_vid_free_urbs(dev);

    /* free gvusb2 */
    gvusb2_free(&dev->gv);

    /* free me */
    kfree(dev);

    return 0;
}

void gvusb2_release(struct v4l2_device *v4l2_dev)
{
    struct gvusb2_vid *dev = container_of(v4l2_dev, struct gvusb2_vid, v4l2_dev);

    gvusb2_dbg(&dev->intf->dev, "releasing gvusb2\n");

    gvusb2_i2c_unregister(dev);

    v4l2_ctrl_handler_free(&dev->ctrl_handler);
    v4l2_device_unregister(&dev->v4l2_dev);

    mutex_destroy(&dev->v4l2_lock);
    mutex_destroy(&dev->vb2q_lock);

    gvusb2_vid_free(dev);
}

int gvusb2_vid_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct usb_device *udev;
    struct gvusb2_vid *dev;
    int i, ret;
    struct usb_endpoint_descriptor *video_ep = NULL;

    gvusb2_dbg(&intf->dev, "gvusb2_vid_probe(intf, id)\n");

    udev = interface_to_usbdev(intf);

    /* check if we're on the video interface */
    for (i = 0; i < intf->num_altsetting; i++) {
        int ep;
        for (ep = 0; ep < intf->altsetting[i].desc.bNumEndpoints; ep++) {
            struct usb_endpoint_descriptor *e =
                &intf->altsetting[i].endpoint[ep].desc;
            if (usb_endpoint_dir_in(e) &&
                    e->bEndpointAddress == 0x82 &&
                    usb_endpoint_xfer_isoc(e) &&
                    /* TODO: change this to better code? */
                    e->wMaxPacketSize == 0x1400) {
                video_ep = e;
                gvusb2_dbg(&intf->dev, "found video at altsetting %d endpoint %d\n",
                    i, ep);
            }
        }
    }

    /* if we don't have a video device, we don't accept */
    if (video_ep == NULL) {
        return -ENODEV;
    }

    /* allocate our driver data */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL) {
        return -ENOMEM;
    }

    /* initialize gvusb2 core stuff */
    ret = gvusb2_init(&dev->gv, udev);
    if (ret < 0) {
        goto free_dev;
    }

    /* XXX: No hardcoding here. */
    ret = usb_set_interface(udev, 0, 5);
    if (ret < 0) {
        goto free_gvusb2;
    }

    /* initialize gvusb2_vid data */
    dev->ep = video_ep;
    dev->intf = intf;

    /* initialize the stk1150 in the gvusb2 */
    gvusb2_stk1150_init(dev);

    /* allocate URBs */
    gvusb2_vid_allocate_urbs(dev);
    if (ret < 0) {
        goto free_gvusb2;
    }

    /* initialize i2c data */
    ret = gvusb2_i2c_register(dev);
    if (ret < 0) {
        goto free_urbs;
    }

    /* initialize video stuff */
    ret = gvusb2_vb2_setup(dev);
    if (ret < 0) {
        goto unregister_i2c;
    }
    ret = gvusb2_v4l2_register(dev);
    if (ret < 0) {
        goto unregister_i2c;
    }
    ret = gvusb2_video_register(dev);
    if (ret < 0) {
        goto unregister_v4l2;
    }

    /* attach our data to the interface */
    usb_set_intfdata(intf, dev);

    return 0;

unregister_v4l2:
    gvusb2_v4l2_unregister(dev);
unregister_i2c:
    gvusb2_i2c_unregister(dev);
free_urbs:
    gvusb2_vid_free_urbs(dev);
free_gvusb2:
    gvusb2_free(&dev->gv);
free_dev:
    kfree(dev);

    return ret;
}

void gvusb2_vid_disconnect(struct usb_interface *intf)
{
    struct gvusb2_vid *dev;

    gvusb2_dbg(&intf->dev, "gvusb2_vid_disconnect(intf)\n");

    /* remove our data from the interface */
    dev = usb_get_intfdata(intf);
    usb_set_intfdata(intf, NULL);

    mutex_lock(&dev->vb2q_lock);
    mutex_lock(&dev->v4l2_lock);

    /* cancel urbs */
    gvusb2_vid_cancel_urbs(dev);
    gvusb2_vid_free_urbs(dev);

    /* clear buffer list queue */
    gvusb2_vid_clear_queue(dev);

    /* clean up v4l2 */
    video_unregister_device(&dev->vdev);
    v4l2_device_disconnect(&dev->v4l2_dev);

    /* make udev NULL? */

    mutex_unlock(&dev->v4l2_lock);
    mutex_unlock(&dev->vb2q_lock);

    /* decrease v4l2 refcount */
    v4l2_device_put(&dev->v4l2_dev);
}

static struct usb_driver gvusb2_vid_usb_driver = {
    .name = "gvusb2-vid",
    .probe = gvusb2_vid_probe,
    .disconnect = gvusb2_vid_disconnect,
    .id_table = gvusb2_id_table,
};

module_usb_driver(gvusb2_vid_usb_driver);
