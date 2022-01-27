// SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include "gvusb2.h"

#define CARD_NAME "gvusb2"

MODULE_DESCRIPTION("gvusb2 sound driver");
MODULE_AUTHOR("Isaac Lozano <109lozanoi@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *ids[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enabled[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;


static const struct usb_device_id gvusb2_id_table[] = {
	{ USB_DEVICE(GVUSB2_VENDOR_ID, GVUSB2_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, gvusb2_id_table);

struct gvusb2_snd {
	struct gvusb2_dev gv;
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep;

	/* urb */
	struct urb *urbs[GVUSB2_NUM_URBS];

	/* alsa */
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	int dma_offset;
	int avail;
	int hw_ptr;
	spinlock_t lock;
};

static struct snd_pcm_hardware gvusb2_snd_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = (128 * 1024),
	.period_bytes_min = 0xc000,
	.period_bytes_max = (128 * 1024),
	.periods_min = 1,
	.periods_max = 32,
};

/* predefines */
static int gvusb2_snd_submit_isoc(struct gvusb2_snd *dev);
static void gvusb2_snd_cancel_isoc(struct gvusb2_snd *dev);

/*****************************************************************************
 *  Alsa Stuff
 ****************************************************************************/

void gvusb2_snd_process_pcm(
	struct gvusb2_snd *dev,
	unsigned char *buf,
	unsigned int len)
{
	unsigned long flags;
	struct snd_pcm_runtime *runtime = dev->substream->runtime;
	int frames = bytes_to_frames(runtime, len);

	spin_lock_irqsave(&dev->lock, flags);
	dev->hw_ptr += frames;
	if (dev->hw_ptr >= runtime->buffer_size)
		dev->hw_ptr -= runtime->buffer_size;

	dev->avail += frames;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (dev->dma_offset + len > runtime->dma_bytes) {
		int len_to_copy = runtime->dma_bytes - dev->dma_offset;

		memcpy(runtime->dma_area + dev->dma_offset, buf, len_to_copy);

		len -= len_to_copy;
		buf += len_to_copy;
		dev->dma_offset = 0;
	}

	memcpy(runtime->dma_area + dev->dma_offset, buf, len);
	dev->dma_offset += len;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->avail >= runtime->period_size) {
		dev->avail -= runtime->period_size;
		spin_unlock_irqrestore(&dev->lock, flags);
		snd_pcm_period_elapsed(dev->substream);
		return;
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static int gvusb2_snd_capture_open(struct snd_pcm_substream *substream)
{
	int ret;
	unsigned long flags;
	struct gvusb2_snd *dev = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	ret = gvusb2_snd_submit_isoc(dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->substream == NULL) {
		dev->substream = substream;
		runtime->hw = gvusb2_snd_hw;
		ret = 0;
	} else {
		ret = -EBUSY;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	return ret;
}

static int gvusb2_snd_capture_close(struct snd_pcm_substream *substream)
{
	struct gvusb2_snd *dev = snd_pcm_substream_chip(substream);

	dev->substream = NULL;
	gvusb2_snd_cancel_isoc(dev);

	return 0;
}

static int gvusb2_snd_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params)
{
	unsigned int bytes;

	bytes = params_buffer_bytes(hw_params);
	if (substream->runtime->dma_bytes > 0)
		vfree(substream->runtime->dma_area);

	substream->runtime->dma_bytes = 0;
	substream->runtime->dma_area = vmalloc(bytes);
	if (substream->runtime->dma_area == NULL)
		return -ENOMEM;

	substream->runtime->dma_bytes = bytes;

	return 0;
}

static int gvusb2_snd_hw_free(struct snd_pcm_substream *substream)
{
	if (substream->runtime->dma_bytes > 0)
		vfree(substream->runtime->dma_area);

	substream->runtime->dma_bytes = 0;

	return 0;
}

static int gvusb2_snd_pcm_prepare(struct snd_pcm_substream *substream)
{
	/* TODO: Do we need this? */
	return 0;
}

/* NOTE: THIS IS ATOMIC */
static int gvusb2_snd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct gvusb2_snd *dev = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		dev->dma_offset = 0;
		dev->hw_ptr = 0;
		dev->avail = 0;
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

/* NOTE: THIS IS ATOMIC */
static snd_pcm_uframes_t gvusb2_snd_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	struct gvusb2_snd *dev = snd_pcm_substream_chip(substream);

	return dev->hw_ptr;
}

static struct page *gvusb2_snd_pcm_page(
	struct snd_pcm_substream *substream,
	unsigned long offset)
{
	return vmalloc_to_page(substream->runtime->dma_area + offset);
}

static int gvusb2_snd_dev_free(struct snd_device *device)
{
	/* deallocate all sound card device stuff */
	/* TODO: do we need this? */

	return 0;
}

static struct snd_device_ops gvusb2_snd_device_ops = {
	.dev_free = gvusb2_snd_dev_free,
};

static const struct snd_pcm_ops gvusb2_snd_capture_ops = {
	.open      = gvusb2_snd_capture_open,
	.close     = gvusb2_snd_capture_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = gvusb2_snd_hw_params,
	.hw_free   = gvusb2_snd_hw_free,
	.prepare   = gvusb2_snd_pcm_prepare,
	.trigger   = gvusb2_snd_pcm_trigger,
	.pointer   = gvusb2_snd_pcm_pointer,
	.page      = gvusb2_snd_pcm_page,
};

int gvusb2_snd_alsa_init(struct gvusb2_snd *dev)
{
	int ret;
	int crdIdx;

	if(crdIdx >= SNDRV_CARDS)
		return -ENODEV;
		
	if(!enabled[crdIdx]) {
		crdIdx++;
		return -ENOENT;
	}

	spin_lock_init(&dev->lock);
	dev->hw_ptr = dev->dma_offset = dev->avail = 0;

	ret = snd_card_new(&dev->intf->dev, index[crdIdx], ids[crdIdx], THIS_MODULE, 0,
			&dev->card);
	if (ret < 0)
		return ret;

	ret = snd_device_new(dev->card, SNDRV_DEV_LOWLEVEL, dev,
		&gvusb2_snd_device_ops);
	if (ret < 0) {
		snd_card_free(dev->card);
		return ret;
	}

	ret = snd_pcm_new(dev->card, "analog in", 0, 0, 1, &dev->pcm);
	if (ret < 0) {
		snd_card_free(dev->card);
		return ret;
	}

	dev->pcm->private_data = dev;

	snd_pcm_set_ops(dev->pcm, SNDRV_PCM_STREAM_CAPTURE,
		&gvusb2_snd_capture_ops);

	strscpy(dev->card->shortname, "gvusb2", sizeof(dev->card->shortname));
	strscpy(dev->card->longname, "gvusb2", sizeof(dev->card->longname));
	strscpy(dev->card->driver, "gvusb2-snd", sizeof(dev->card->driver));

	/* register the card */
	ret = snd_card_register(dev->card);
	if (ret < 0) {
		snd_card_free(dev->card);
		return ret;
	}

	return 0;
}

static void gvusb2_snd_alsa_free(struct gvusb2_snd *dev)
{
	snd_card_free(dev->card);
	dev->card = NULL;
}

/*****************************************************************************
 * USB Stuff
 ****************************************************************************/

/* Do not call in atomic contexts */
void gvusb2_snd_cancel_isoc(struct gvusb2_snd *dev)
{
	int i;

	for (i = 0; i < GVUSB2_NUM_URBS; i++) {
		struct urb *urb = dev->urbs[i];

		if (urb != NULL)
			usb_kill_urb(dev->urbs[i]);
	}
}

void gvusb2_snd_free_isoc(struct gvusb2_snd *dev)
{
	int i;

	for (i = 0; i < GVUSB2_NUM_URBS; i++) {
		struct urb *urb = dev->urbs[i];

		if (urb != NULL) {
			if (urb->transfer_buffer != NULL)
				kfree(urb->transfer_buffer);
			usb_free_urb(urb);
			dev->urbs[i] = NULL;
		}
	}
}

void gvusb2_snd_process_isoc(struct gvusb2_snd *dev, struct urb *urb)
{
	int i;
	unsigned char *buf_iter;

	if (dev->substream == NULL) {
		gvusb2_dbg(&dev->intf->dev, "substream is null, skipping processing\n");
		return;
	}

	buf_iter = urb->transfer_buffer;
	for (i = 0; i < urb->number_of_packets; i++) {
		if (urb->iso_frame_desc[i].status < 0) {
			gvusb2_dbg(&dev->intf->dev, "bad iso packet. skipping.\n");
		} else {
			gvusb2_snd_process_pcm(dev,
				buf_iter, urb->iso_frame_desc[i].actual_length);
		}

		buf_iter += urb->iso_frame_desc[i].length;
	}
}

static void gvusb2_snd_isoc_irq(struct urb *urb)
{
	int i, ret;
	struct gvusb2_snd *dev = urb->context;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		gvusb2_dbg(&dev->intf->dev, "urb error! status %d\n",
			urb->status);
		return;
	}

	gvusb2_snd_process_isoc(dev, urb);

	for (i = 0; i < urb->number_of_packets; i++) {
		urb->iso_frame_desc[i].status = 0;
		urb->iso_frame_desc[i].actual_length = 0;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		gvusb2_dbg(&dev->intf->dev, "urb resubmit failed (%d)\n", ret);
}

static int gvusb2_snd_submit_isoc(struct gvusb2_snd *dev)
{
	int i, ret;

	for (i = 0; i < GVUSB2_NUM_URBS; i++) {
		ret = usb_submit_urb(dev->urbs[i], GFP_KERNEL);
		if (ret < 0)
			/* TODO: clean up */
			gvusb2_dbg(&dev->intf->dev,
				"error submitting urb %d\n", ret);
	}

	return 0;
}

static int gvusb2_snd_allocate_urbs(struct gvusb2_snd *dev)
{
	int i;

	for (i = 0; i < GVUSB2_NUM_URBS; i++) {
		struct urb *urb;
		int total_offset, pidx;

		urb = usb_alloc_urb(GVUSB2_NUM_ISOCH_PACKETS, GFP_KERNEL);
		if (urb == NULL)
			goto free_urbs;

		dev->urbs[i] = urb;

		urb->transfer_buffer = kzalloc(
			GVUSB2_NUM_ISOCH_PACKETS * GVUSB2_MAX_AUDIO_PACKET_SIZE,
			GFP_KERNEL);
		if (urb->transfer_buffer == NULL)
			goto free_urbs;

		urb->dev = dev->gv.udev;
		urb->pipe = usb_rcvisocpipe(dev->gv.udev, 0x04);
		urb->transfer_buffer_length =
			GVUSB2_NUM_ISOCH_PACKETS * GVUSB2_MAX_AUDIO_PACKET_SIZE;
		urb->complete = gvusb2_snd_isoc_irq;
		urb->context = dev;
		urb->interval = 1;
		urb->start_frame = 0;
		urb->number_of_packets = GVUSB2_NUM_ISOCH_PACKETS;
		urb->transfer_flags = URB_ISO_ASAP;

		total_offset = 0;
		for (pidx = 0; pidx < GVUSB2_NUM_ISOCH_PACKETS; pidx++) {
			urb->iso_frame_desc[pidx].offset = total_offset;
			urb->iso_frame_desc[pidx].length =
				GVUSB2_MAX_AUDIO_PACKET_SIZE;
			total_offset += GVUSB2_MAX_AUDIO_PACKET_SIZE;
		}
	}

	return 0;

free_urbs:
	gvusb2_snd_free_isoc(dev);
	return -ENOMEM;
}

/*****************************************************************************
 * Driver Stuff
 ****************************************************************************/

int gvusb2_snd_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev;
	struct gvusb2_snd *dev;
	int i, ret;
	struct usb_endpoint_descriptor *audio_ep = NULL;

	udev = interface_to_usbdev(intf);

	/*
	 * The GV-USB2 uses a proprietary audio transfer, but the chip has
	 * support for the USB audio class, so let's reject it if we see it.
	 */
	if (intf->altsetting[0].desc.bInterfaceClass == USB_CLASS_AUDIO)
		return -ENODEV;

	/* check if we're on the audio interface */
	for (i = 0; i < intf->num_altsetting; i++) {
		int ep;
		int num_endpoints = intf->altsetting[i].desc.bNumEndpoints;

		for (ep = 0; ep < num_endpoints; ep++) {
			struct usb_endpoint_descriptor *e =
				&intf->altsetting[i].endpoint[ep].desc;
			if (usb_endpoint_dir_in(e) &&
					e->bEndpointAddress == 0x84 &&
					usb_endpoint_xfer_isoc(e) &&
					e->wMaxPacketSize == 0x100) {
				audio_ep = e;
				gvusb2_dbg(&intf->dev, "found audio at altsetting %d endpoint %d\n",
					i, ep);
			}
		}
	}

	/* if we don't have an audio device, we don't accept */
	if (audio_ep == NULL)
		return -ENODEV;

	/* allocate our driver data */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;

	/* initialize gvusb2 core stuff */
	ret = gvusb2_init(&dev->gv, udev);
	if (ret < 0)
		goto free_dev;

	/* XXX: No hardcoding here. */
	ret = usb_set_interface(udev, 2, 1);
	if (ret < 0)
		goto free_gvusb2;

	/* reset the adc */
	ret = gvusb2_snd_reset_adc(&dev->gv);
	if (ret < 0)
		goto free_gvusb2;

	/* initialize gvusb2_snd data */
	dev->ep = audio_ep;
	dev->intf = intf;

	/* initialize sound stuff */
	ret = gvusb2_snd_alsa_init(dev);
	if (ret < 0)
		goto free_gvusb2;

	/* allocate URBs */
	ret = gvusb2_snd_allocate_urbs(dev);
	if (ret < 0)
		goto free_alsa;

	/* attach our data to the interface */
	usb_set_intfdata(intf, dev);

	return 0;

free_alsa:
	gvusb2_snd_alsa_free(dev);

free_gvusb2:
	gvusb2_free(&dev->gv);

free_dev:
	kfree(dev);

	return ret;
}

void gvusb2_snd_disconnect(struct usb_interface *intf)
{
	struct gvusb2_snd *dev;

	/* remove our data from the interface */
	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	/* free isoc urbs */
	gvusb2_snd_free_isoc(dev);

	/* free the sound card */
	gvusb2_snd_alsa_free(dev);

	/* free the internal gvusb2 device */
	gvusb2_free(&dev->gv);

	/* free me */
	kfree(dev);
}

static struct usb_driver gvusb2_snd_usb_driver = {
	.name = "gvusb2-snd",
	.probe = gvusb2_snd_probe,
	.disconnect = gvusb2_snd_disconnect,
	.id_table = gvusb2_id_table,
};

module_usb_driver(gvusb2_snd_usb_driver);
