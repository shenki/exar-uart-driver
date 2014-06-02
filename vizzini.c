/*
 * vizzini.c
 *
 * Copyright (c) 2013 Exar Corporation, Inc.
 *
 * ChangeLog:
 * 
 *            v1.0- Support for Kernel 3.5 and newer,
 *		    based on cdc-acm.c sample of 3.6 kernel.
 */


/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/list.h>

#include "vizzini.h"
#include "vzioctl.h"

#define DRIVER_AUTHOR "Ravi Reddy"
#define DRIVER_DESC "Exar USB UART Driver for XR21V141x "

static struct usb_driver xr21v141x_driver;
static struct tty_driver *xr21v141x_tty_driver;
static struct xr21v141x *xr21v141x_table[XR21V141X_TTY_MINORS];

static DEFINE_MUTEX(xr21v141x_table_lock);

/*
 * Functions for ACM control messages.
 */

static int xr21v141x_ctrl_msg(struct xr21v141x *xr21v141x, int request, int value,
							void *buf, int len)
{
	int retval = usb_control_msg(xr21v141x->dev, usb_sndctrlpipe(xr21v141x->dev, 0),
		request, USB_RT_ACM, value,
		xr21v141x->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);
	dev_dbg(&xr21v141x->control->dev,
			"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
			__func__, request, value, len, retval);
	return retval < 0 ? retval : 0;
}

#define xr21v141x_set_control(xr21v141x, control) \
	xr21v141x_ctrl_msg(xr21v141x, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0)
#define xr21v141x_set_line(xr21v141x, line) \
	xr21v141x_ctrl_msg(xr21v141x, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define xr21v141x_send_break(xr21v141x, ms) \
	xr21v141x_ctrl_msg(xr21v141x, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int xr21v141x_wb_alloc(struct xr21v141x *xr21v141x)
{
	int i, wbn;
	struct xr21v141x_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &xr21v141x->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}

static int xr21v141x_wb_is_avail(struct xr21v141x *xr21v141x)
{
	int i, n;
	unsigned long flags;

	n = ACM_NW;
	spin_lock_irqsave(&xr21v141x->write_lock, flags);
	for (i = 0; i < ACM_NW; i++)
		n -= xr21v141x->wb[i].use;
	spin_unlock_irqrestore(&xr21v141x->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold xr21v141x->write_lock
 */
static void xr21v141x_write_done(struct xr21v141x *xr21v141x, struct xr21v141x_wb *wb)
{
	wb->use = 0;
	xr21v141x->transmitting--;
	usb_autopm_put_interface_async(xr21v141x->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int xr21v141x_start_wb(struct xr21v141x *xr21v141x, struct xr21v141x_wb *wb)
{
	int rc;

	xr21v141x->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = xr21v141x->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&xr21v141x->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		xr21v141x_write_done(xr21v141x, wb);
	}
	return rc;
}

static int xr21v141x_write_start(struct xr21v141x *xr21v141x, int wbn)
{
	unsigned long flags;
	struct xr21v141x_wb *wb = &xr21v141x->wb[wbn];
	int rc;

	spin_lock_irqsave(&xr21v141x->write_lock, flags);
	if (!xr21v141x->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&xr21v141x->write_lock, flags);
		return -ENODEV;
	}

	dev_vdbg(&xr21v141x->data->dev, "%s - susp_count %d\n", __func__,
							xr21v141x->susp_count);
	usb_autopm_get_interface_async(xr21v141x->control);
	if (xr21v141x->susp_count) {
		if (!xr21v141x->delayed_wb)
			xr21v141x->delayed_wb = wb;
		else
			usb_autopm_put_interface_async(xr21v141x->control);
		spin_unlock_irqrestore(&xr21v141x->write_lock, flags);
		return 0;	/* A white lie */
	}
	usb_mark_last_busy(xr21v141x->dev);

	rc = xr21v141x_start_wb(xr21v141x, wb);
	spin_unlock_irqrestore(&xr21v141x->write_lock, flags);

	return rc;

}
/*
 * attributes exported through sysfs
 */
static ssize_t show_caps
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);

	return sprintf(buf, "%d", xr21v141x->ctrl_caps);
}
static DEVICE_ATTR(bmCapabilities, S_IRUGO, show_caps, NULL);

static ssize_t show_country_codes
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);

	memcpy(buf, xr21v141x->country_codes, xr21v141x->country_code_size);
	return xr21v141x->country_code_size;
}

static DEVICE_ATTR(wCountryCodes, S_IRUGO, show_country_codes, NULL);

static ssize_t show_country_rel_date
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);

	return sprintf(buf, "%d", xr21v141x->country_rel_date);
}

static DEVICE_ATTR(iCountryCodeRelDate, S_IRUGO, show_country_rel_date, NULL);
/*
 * Interrupt handlers for various ACM device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void xr21v141x_ctrl_irq(struct urb *urb)
{
	struct xr21v141x *xr21v141x = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	struct tty_struct *tty;
	unsigned char *data;
	int newctrl;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&xr21v141x->control->dev,
				"%s - urb shutting down with status: %d\n",
				__func__, status);
		return;
	default:
		dev_dbg(&xr21v141x->control->dev,
				"%s - nonzero urb status received: %d\n",
				__func__, status);
		goto exit;
	}

	usb_mark_last_busy(xr21v141x->dev);

	data = (unsigned char *)(dr + 1);
	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		dev_dbg(&xr21v141x->control->dev, "%s - network connection: %d\n",
							__func__, dr->wValue);
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		tty = tty_port_tty_get(&xr21v141x->port);
		newctrl = get_unaligned_le16(data);

		if (tty) {
			if (!xr21v141x->clocal &&
				(xr21v141x->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
				dev_dbg(&xr21v141x->control->dev,
					"%s - calling hangup\n", __func__);
				tty_hangup(tty);
			}
			tty_kref_put(tty);
		}

		xr21v141x->ctrlin = newctrl;

		dev_dbg(&xr21v141x->control->dev,
			"%s - input control lines: dcd%c dsr%c break%c "
			"ring%c framing%c parity%c overrun%c\n",
			__func__,
			xr21v141x->ctrlin & ACM_CTRL_DCD ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_DSR ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_BRK ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_RI  ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_FRAMING ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_PARITY ? '+' : '-',
			xr21v141x->ctrlin & ACM_CTRL_OVERRUN ? '+' : '-');
			break;

	default:
		dev_dbg(&xr21v141x->control->dev,
			"%s - unknown notification %d received: index %d "
			"len %d data0 %d data1 %d\n",
			__func__,
			dr->bNotificationType, dr->wIndex,
			dr->wLength, data[0], data[1]);
		break;
	}
exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&xr21v141x->control->dev, "%s - usb_submit_urb failed: %d\n",
							__func__, retval);
}

static int xr21v141x_submit_read_urb(struct xr21v141x *xr21v141x, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &xr21v141x->read_urbs_free))
		return 0;

	dev_vdbg(&xr21v141x->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(xr21v141x->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM) {
			dev_err(&xr21v141x->data->dev,
					"%s - usb_submit_urb failed: %d\n",
					__func__, res);
		}
		set_bit(index, &xr21v141x->read_urbs_free);
		return res;
	}

	return 0;
}

static int xr21v141x_submit_read_urbs(struct xr21v141x *xr21v141x, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < xr21v141x->rx_buflimit; ++i) {
		res = xr21v141x_submit_read_urb(xr21v141x, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void xr21v141x_process_read_urb(struct xr21v141x *xr21v141x, struct urb *urb)
{
	if (!urb->actual_length)
		return;

	tty_insert_flip_string(&xr21v141x->port, urb->transfer_buffer, urb->actual_length);
	tty_flip_buffer_push(&xr21v141x->port);
}

static void xr21v141x_read_bulk_callback(struct urb *urb)
{
	struct xr21v141x_rb *rb = urb->context;
	struct xr21v141x *xr21v141x = rb->instance;
	unsigned long flags;

	dev_vdbg(&xr21v141x->data->dev, "%s - urb %d, len %d\n", __func__,
					rb->index, urb->actual_length);
	set_bit(rb->index, &xr21v141x->read_urbs_free);

	if (!xr21v141x->dev) {
		dev_dbg(&xr21v141x->data->dev, "%s - disconnected\n", __func__);
		return;
	}
	usb_mark_last_busy(xr21v141x->dev);

	if (urb->status) {
		dev_dbg(&xr21v141x->data->dev, "%s - non-zero urb status: %d\n",
							__func__, urb->status);
		return;
	}
	xr21v141x_process_read_urb(xr21v141x, urb);

	/* throttle device if requested by tty */
	spin_lock_irqsave(&xr21v141x->read_lock, flags);
	xr21v141x->throttled = xr21v141x->throttle_req;
	if (!xr21v141x->throttled && !xr21v141x->susp_count) {
		spin_unlock_irqrestore(&xr21v141x->read_lock, flags);
		xr21v141x_submit_read_urb(xr21v141x, rb->index, GFP_ATOMIC);
	} else {
		spin_unlock_irqrestore(&xr21v141x->read_lock, flags);
	}
}

/* data interface wrote those outgoing bytes */
static void xr21v141x_write_bulk(struct urb *urb)
{
	struct xr21v141x_wb *wb = urb->context;
	struct xr21v141x *xr21v141x = wb->instance;
	unsigned long flags;

	if (urb->status	|| (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&xr21v141x->data->dev, "%s - len %d/%d, status %d\n",
			__func__,
			urb->actual_length,
			urb->transfer_buffer_length,
			urb->status);

	spin_lock_irqsave(&xr21v141x->write_lock, flags);
	xr21v141x_write_done(xr21v141x, wb);
	spin_unlock_irqrestore(&xr21v141x->write_lock, flags);
	schedule_work(&xr21v141x->work);
}

static void xr21v141x_softint(struct work_struct *work)
{
	struct xr21v141x *xr21v141x = container_of(work, struct xr21v141x, work);
	struct tty_struct *tty;

	dev_vdbg(&xr21v141x->data->dev, "%s\n", __func__);

	tty = tty_port_tty_get(&xr21v141x->port);
	if (!tty)
		return;
	tty_wakeup(tty);
	tty_kref_put(tty);
}

/*
 * TTY handlers
 */

static struct xr21v141x *xr21v141x_get_by_index(unsigned index)
{
	struct xr21v141x *xr21v141x;

	mutex_lock(&xr21v141x_table_lock);
	xr21v141x = xr21v141x_table[index];
	if (xr21v141x) {
		mutex_lock(&xr21v141x->mutex);
		if (xr21v141x->disconnected) {
			mutex_unlock(&xr21v141x->mutex);
			xr21v141x = NULL;
		} else {
			tty_port_get(&xr21v141x->port);
			mutex_unlock(&xr21v141x->mutex);
		}
	}
	mutex_unlock(&xr21v141x_table_lock);
	return xr21v141x;
}

/*
 * Try to find an available minor number and if found, associate it with 'xr21v141x'.
 */
static int xr21v141x_alloc_minor(struct xr21v141x *xr21v141x)
{
	int minor;

	mutex_lock(&xr21v141x_table_lock);
	for (minor = 0; minor < XR21V141X_TTY_MINORS; minor++) {
		if (!xr21v141x_table[minor]) {
			xr21v141x_table[minor] = xr21v141x;
			break;
		}
	}
	mutex_unlock(&xr21v141x_table_lock);

	return minor;
}

/* Release the minor number associated with 'xr21v141x'.  */
static void xr21v141x_release_minor(struct xr21v141x *xr21v141x)
{
	mutex_lock(&xr21v141x_table_lock);
	xr21v141x_table[xr21v141x->minor] = NULL;
	mutex_unlock(&xr21v141x_table_lock);
}

static int xr21v141x_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x;
	int retval;

	dev_dbg(tty->dev, "%s\n", __func__);

	xr21v141x = xr21v141x_get_by_index(tty->index);
	if (!xr21v141x)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = xr21v141x;

	return 0;

error_init_termios:
	tty_port_put(&xr21v141x->port);
	return retval;
}

static int xr21v141x_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct xr21v141x *xr21v141x = tty->driver_data;

	dev_dbg(tty->dev, "%s\n", __func__);

	return tty_port_open(&xr21v141x->port, tty, filp);
}

static int xr21v141x_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = container_of(port, struct xr21v141x, port);
	int retval = -ENODEV;

	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);

	mutex_lock(&xr21v141x->mutex);
	if (xr21v141x->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(xr21v141x->control);
	if (retval)
		goto error_get_interface;

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	xr21v141x->control->needs_remote_wakeup = 1;

	xr21v141x->ctrlurb->dev = xr21v141x->dev;
	if (usb_submit_urb(xr21v141x->ctrlurb, GFP_KERNEL)) {
		dev_err(&xr21v141x->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	xr21v141x->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS;
	if (xr21v141x_set_control(xr21v141x, xr21v141x->ctrlout) < 0 &&
	    (xr21v141x->ctrl_caps & USB_CDC_CAP_LINE))
		goto error_set_control;

	usb_autopm_put_interface(xr21v141x->control);

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	spin_lock_irq(&xr21v141x->read_lock);
	xr21v141x->throttled = 0;
	xr21v141x->throttle_req = 0;
	spin_unlock_irq(&xr21v141x->read_lock);

	if (xr21v141x_submit_read_urbs(xr21v141x, GFP_KERNEL))
		goto error_submit_read_urbs;

	mutex_unlock(&xr21v141x->mutex);

	return 0;

error_submit_read_urbs:
	xr21v141x->ctrlout = 0;
	xr21v141x_set_control(xr21v141x, xr21v141x->ctrlout);
error_set_control:
	usb_kill_urb(xr21v141x->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(xr21v141x->control);
error_get_interface:
disconnected:
	mutex_unlock(&xr21v141x->mutex);
	return retval;
}

static void xr21v141x_port_destruct(struct tty_port *port)
{
	struct xr21v141x *xr21v141x = container_of(port, struct xr21v141x, port);

	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);

	tty_unregister_device(xr21v141x_tty_driver, xr21v141x->minor);
	xr21v141x_release_minor(xr21v141x);
	usb_put_intf(xr21v141x->control);
	kfree(xr21v141x->country_codes);
	kfree(xr21v141x);
}

static void xr21v141x_port_shutdown(struct tty_port *port)
{
	struct xr21v141x *xr21v141x = container_of(port, struct xr21v141x, port);
	int i;

	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);

	mutex_lock(&xr21v141x->mutex);
	if (!xr21v141x->disconnected) {
		usb_autopm_get_interface(xr21v141x->control);
		xr21v141x_set_control(xr21v141x, xr21v141x->ctrlout = 0);
		usb_kill_urb(xr21v141x->ctrlurb);
		for (i = 0; i < ACM_NW; i++)
			usb_kill_urb(xr21v141x->wb[i].urb);
		for (i = 0; i < xr21v141x->rx_buflimit; i++)
			usb_kill_urb(xr21v141x->read_urbs[i]);
		xr21v141x->control->needs_remote_wakeup = 0;
		usb_autopm_put_interface(xr21v141x->control);
	}
	mutex_unlock(&xr21v141x->mutex);
}

static void xr21v141x_tty_cleanup(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);
	tty_port_put(&xr21v141x->port);
}

static void xr21v141x_tty_hangup(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);
	tty_port_hangup(&xr21v141x->port);
}

static void xr21v141x_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);
	tty_port_close(&xr21v141x->port, tty, filp);
}

static int xr21v141x_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct xr21v141x_wb *wb;

	if (!count)
		return 0;

	dev_vdbg(&xr21v141x->data->dev, "%s - count %d\n", __func__, count);

	spin_lock_irqsave(&xr21v141x->write_lock, flags);
	wbn = xr21v141x_wb_alloc(xr21v141x);
	if (wbn < 0) {
		spin_unlock_irqrestore(&xr21v141x->write_lock, flags);
		return 0;
	}
	wb = &xr21v141x->wb[wbn];

	count = (count > xr21v141x->writesize) ? xr21v141x->writesize : count;
	dev_vdbg(&xr21v141x->data->dev, "%s - write %d\n", __func__, count);
	memcpy(wb->buf, buf, count);
	wb->len = count;
	spin_unlock_irqrestore(&xr21v141x->write_lock, flags);

	stat = xr21v141x_write_start(xr21v141x, wbn);
	if (stat < 0)
		return stat;
	return count;
}

static int xr21v141x_tty_write_room(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	
	return xr21v141x_wb_is_avail(xr21v141x) ? xr21v141x->writesize : 0;
}

static int xr21v141x_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	/*
	 * if the device was unplugged then any remaining characters fell out
	 * of the connector ;)
	 */
	if (xr21v141x->disconnected)
		return 0;
	
	return (ACM_NW - xr21v141x_wb_is_avail(xr21v141x)) * xr21v141x->writesize;
}

static void xr21v141x_tty_throttle(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;

	spin_lock_irq(&xr21v141x->read_lock);
	xr21v141x->throttle_req = 1;
	spin_unlock_irq(&xr21v141x->read_lock);
}

static void xr21v141x_tty_unthrottle(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	unsigned int was_throttled;

	spin_lock_irq(&xr21v141x->read_lock);
	was_throttled = xr21v141x->throttled;
	xr21v141x->throttled = 0;
	xr21v141x->throttle_req = 0;
	spin_unlock_irq(&xr21v141x->read_lock);

	if (was_throttled)
		xr21v141x_submit_read_urbs(xr21v141x, GFP_KERNEL);
}

static int xr21v141x_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	int retval;

	retval = xr21v141x_send_break(xr21v141x, state ? 0xffff : 0);
	if (retval < 0)
		dev_dbg(&xr21v141x->control->dev, "%s - send break failed\n",
								__func__);
	return retval;
}

static int xr21v141x_tty_tiocmget(struct tty_struct *tty)
{
	struct xr21v141x *xr21v141x = tty->driver_data;

	return (xr21v141x->ctrlout & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
	       (xr21v141x->ctrlout & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
	       (xr21v141x->ctrlin  & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
	       (xr21v141x->ctrlin  & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
	       (xr21v141x->ctrlin  & ACM_CTRL_DCD ? TIOCM_CD  : 0) |
	       TIOCM_CTS;
}

static int xr21v141x_tty_tiocmset(struct tty_struct *tty,
			    unsigned int set, unsigned int clear)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	unsigned int newctrl;

	newctrl = xr21v141x->ctrlout;
	set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (xr21v141x->ctrlout == newctrl)
		return 0;
	return xr21v141x_set_control(xr21v141x, xr21v141x->ctrlout = newctrl);
}

static int get_serial_info(struct xr21v141x *xr21v141x, struct serial_struct __user *info)
{
	struct serial_struct tmp;

	if (!info)
		return -EINVAL;

	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = ASYNC_LOW_LATENCY;
	tmp.xmit_fifo_size = xr21v141x->writesize;
	tmp.baud_base = le32_to_cpu(xr21v141x->line.dwDTERate);
	tmp.close_delay	= xr21v141x->port.close_delay / 10;
	tmp.closing_wait = xr21v141x->port.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
				ASYNC_CLOSING_WAIT_NONE :
				xr21v141x->port.closing_wait / 10;

	if (copy_to_user(info, &tmp, sizeof(tmp)))
		return -EFAULT;
	else
		return 0;
}

static int set_serial_info(struct xr21v141x *xr21v141x,
				struct serial_struct __user *newinfo)
{
	struct serial_struct new_serial;
	unsigned int closing_wait, close_delay;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	close_delay = new_serial.close_delay * 10;
	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
			ASYNC_CLOSING_WAIT_NONE : new_serial.closing_wait * 10;

	mutex_lock(&xr21v141x->port.mutex);

	if (!capable(CAP_SYS_ADMIN)) {
		if ((close_delay != xr21v141x->port.close_delay) ||
		    (closing_wait != xr21v141x->port.closing_wait))
			retval = -EPERM;
		else
			retval = -EOPNOTSUPP;
	} else {
		xr21v141x->port.close_delay  = close_delay;
		xr21v141x->port.closing_wait = closing_wait;
	}

	mutex_unlock(&xr21v141x->port.mutex);
	return retval;
}



static int vizzini_set_reg(struct xr21v141x *xr21v141x,
                           int block, int regnum, int value)
{
        int result;

	dev_dbg(&xr21v141x->control->dev, "%s 0x%02x:0x%02x = 0x%02x\n", __func__, block, regnum, value);

        result = usb_control_msg(xr21v141x->dev,                        /* usb device */
                                 usb_sndctrlpipe(xr21v141x->dev, 0), /* endpoint pipe */
                                 XR_SET_REG,                      /* request */
                                 USB_DIR_OUT | USB_TYPE_VENDOR,   /* request_type */
                                 value,                           /* request value */
                                 regnum | (block << 8),           /* index */
                                 NULL,                            /* data */
                                 0,                               /* size */
                                 5000);                           /* timeout */
        return result;
}


static int vizzini_get_reg(struct xr21v141x *xr21v141x,
                           int block, int reg, char *value)
{
        int result;

        result = usb_control_msg(xr21v141x->dev,                     /* usb device */
                                 usb_rcvctrlpipe(xr21v141x->dev, 0), /* endpoint pipe */
                                 XR_GETN_REG,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 reg | (block << 8),              /* index */
                                 value,                           /* data */
                                 1,                               /* size */
                                 5000);                           /* timeout */

        return result;
}


static void vizzini_disable(struct xr21v141x *xr21v141x)
{
        int block = xr21v141x->block;

        vizzini_set_reg(xr21v141x, block, UART_ENABLE, 0);
        vizzini_set_reg(xr21v141x, URM_REG_BLOCK, URM_ENABLE_BASE + block, 0);
}


static void vizzini_enable(struct xr21v141x *xr21v141x)
{
        int block = xr21v141x->block;

        vizzini_set_reg(xr21v141x, URM_REG_BLOCK, URM_ENABLE_BASE + block, URM_ENABLE_0_TX);
        vizzini_set_reg(xr21v141x, block, UART_ENABLE, UART_ENABLE_TX | UART_ENABLE_RX);
        vizzini_set_reg(xr21v141x, URM_REG_BLOCK, URM_ENABLE_BASE + block, URM_ENABLE_0_TX | URM_ENABLE_0_RX);
}


static void vizzini_loopback(struct xr21v141x *xr21v141x, int from)
{
	int block = xr21v141x->block;
	int lb;

	switch (from)
	{
	case 0: lb = UART_LOOPBACK_CTL_RX_UART0; break;
	case 1: lb = UART_LOOPBACK_CTL_RX_UART1; break;
	case 2: lb = UART_LOOPBACK_CTL_RX_UART2; break;
	case 3: lb = UART_LOOPBACK_CTL_RX_UART3; break;
	default: return;
	}

	dev_info(&xr21v141x->control->dev, "Internal loopback from %d\n", from);

	vizzini_disable(xr21v141x);
	vizzini_set_reg(xr21v141x, block, UART_LOOPBACK_CTL, UART_LOOPBACK_CTL_ENABLE | lb);
	vizzini_enable(xr21v141x);
}

static int vizzini_test_mode(struct xr21v141x *xr21v141x,
                             int selector)
{
        int retval = usb_control_msg(xr21v141x->dev, usb_sndctrlpipe(xr21v141x->dev, 0),
                                     USB_REQ_SET_FEATURE,
                                     USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                     USB_DEVICE_TEST_MODE,
                                     selector << 8,
                                     NULL, 0, 5000);
        dev_dbg(&xr21v141x->control->dev, "vz_test_mode: selector=0x%02x\n", selector);
        return retval < 0 ? retval : 0;
}

static int xr21v141x_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	int rv = -ENOIOCTLCMD;

	unsigned int  block, reg, val, match, preciseflags, unicast, broadcast, flow, selector;
        char	*data;

	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		rv = get_serial_info(xr21v141x, (struct serial_struct __user *) arg);
		break;
	case TIOCSSERIAL:
		rv = set_serial_info(xr21v141x, (struct serial_struct __user *) arg);
		break;

        case VZIOC_GET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;

                data = kmalloc(1, GFP_KERNEL);
                if (data == NULL) {
                        dev_err(&xr21v141x->control->dev, "%s - Cannot allocate USB buffer.\n", __func__);
                        return -ENOMEM;
		}

		if (block == -1)
			block = xr21v141x->block;

                rv = vizzini_get_reg(xr21v141x, block, reg, data);
                if (rv != 1) {
                        dev_err(&xr21v141x->control->dev, "Cannot get register (%d)\n", rv);
                        kfree(data);
                        return -EFAULT;
                }

                if (put_user(data[0], (int __user *)(arg + 2 * sizeof(int)))) {
                        dev_err(&xr21v141x->control->dev, "Cannot put user result\n");
                        kfree(data);
                        return -EFAULT;
                }

                kfree(data);
                break;

        case VZIOC_SET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;
                if (get_user(val, (int __user *)(arg + 2 * sizeof(int))))
                        return -EFAULT;

		if (block == -1)
			block = xr21v141x->block;

                rv = vizzini_set_reg(xr21v141x, block, reg, val);
                if (rv < 0)
                        return -EFAULT;
                break;

	case VZIOC_SET_ADDRESS_MATCH:
                match = arg;

                dev_dbg(&xr21v141x->control->dev, "%s VIOC_SET_ADDRESS_MATCH %d\n", __func__, match);

                vizzini_disable(xr21v141x);

                if (match & VZ_ADDRESS_MATCH_DISABLE) {
                        flow      = UART_FLOW_MODE_NONE;
                } else {
                        flow      = UART_FLOW_MODE_ADDR_MATCH_TX;
                        unicast   = (match >> VZ_ADDRESS_UNICAST_S) & 0xff;
                        broadcast = (match >> VZ_ADDRESS_BROADCAST_S) & 0xff;
                }

                dev_dbg(&xr21v141x->control->dev, "address match: flow=%d ucast=%d bcast=%u\n",
                                   flow, unicast, broadcast);
                vizzini_set_reg(xr21v141x, xr21v141x->block, UART_FLOW, flow);
                vizzini_set_reg(xr21v141x, xr21v141x->block, UART_XON_CHAR, unicast);
                vizzini_set_reg(xr21v141x, xr21v141x->block, UART_XOFF_CHAR, broadcast);

                vizzini_enable(xr21v141x);
                break;

        case VZIOC_SET_PRECISE_FLAGS:
                preciseflags = arg;

                dev_dbg(&xr21v141x->control->dev, "%s VIOC_SET_PRECISE_FLAGS %d\n", __func__, preciseflags);

                vizzini_disable(xr21v141x);

                if (preciseflags) {
                        xr21v141x->preciseflags = 1;
                } else {
                        xr21v141x->preciseflags = 0;
                }

                vizzini_set_reg(xr21v141x, EPLOCALS_REG_BLOCK,
                                (xr21v141x->block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE,
                                xr21v141x->preciseflags);

                vizzini_enable(xr21v141x);
		rv = 0;
                break;

        case VZIOC_TEST_MODE:
                selector = arg;
                dev_dbg(&xr21v141x->control->dev, "%s VIOC_TEST_MODE 0x%02x\n", __func__, selector);
                vizzini_test_mode(xr21v141x, selector);
		rv = 0;
                break;

	case VZIOC_LOOPBACK:
		selector = arg;
		dev_dbg(&xr21v141x->control->dev, "VIOC_LOOPBACK 0x%02x\n", selector);
		vizzini_loopback(xr21v141x, selector);
		rv = 0;
		break;
	}

	return rv;
}

struct vizzini_baud_rate
{
	unsigned int tx;
	unsigned int rx0;
	unsigned int rx1;
};

static struct vizzini_baud_rate vizzini_baud_rates[] = {
	{ 0x000, 0x000, 0x000 },
	{ 0x000, 0x000, 0x000 },
	{ 0x100, 0x000, 0x100 },
	{ 0x020, 0x400, 0x020 },
	{ 0x010, 0x100, 0x010 },
	{ 0x208, 0x040, 0x208 },
	{ 0x104, 0x820, 0x108 },
	{ 0x844, 0x210, 0x884 },
	{ 0x444, 0x110, 0x444 },
	{ 0x122, 0x888, 0x224 },
	{ 0x912, 0x448, 0x924 },
	{ 0x492, 0x248, 0x492 },
	{ 0x252, 0x928, 0x292 },
	{ 0X94A, 0X4A4, 0XA52 },
	{ 0X52A, 0XAA4, 0X54A },
	{ 0XAAA, 0x954, 0X4AA },
	{ 0XAAA, 0x554, 0XAAA },
	{ 0x555, 0XAD4, 0X5AA },
	{ 0XB55, 0XAB4, 0X55A },
	{ 0X6B5, 0X5AC, 0XB56 },
	{ 0X5B5, 0XD6C, 0X6D6 },
	{ 0XB6D, 0XB6A, 0XDB6 },
	{ 0X76D, 0X6DA, 0XBB6 },
	{ 0XEDD, 0XDDA, 0X76E },
	{ 0XDDD, 0XBBA, 0XEEE },
	{ 0X7BB, 0XF7A, 0XDDE },
	{ 0XF7B, 0XEF6, 0X7DE },
	{ 0XDF7, 0XBF6, 0XF7E },
	{ 0X7F7, 0XFEE, 0XEFE },
	{ 0XFDF, 0XFBE, 0X7FE },
	{ 0XF7F, 0XEFE, 0XFFE },
	{ 0XFFF, 0XFFE, 0XFFD },
};

static int vizzini_set_baud_rate(struct xr21v141x *xr21v141x, unsigned int rate)
{
	int 		block 	= xr21v141x->block;
	unsigned int 	divisor = 48000000 / rate;
	unsigned int 	i 	= ((32 * 48000000) / rate) & 0x1f;
	unsigned int 	tx_mask = vizzini_baud_rates[i].tx;
	unsigned int 	rx_mask = (divisor & 1) ? vizzini_baud_rates[i].rx1 : vizzini_baud_rates[i].rx0;

	dev_dbg(&xr21v141x->control->dev, "Setting baud rate to %d: i=%u div=%u tx=%03x rx=%03x\n", rate, i, divisor, tx_mask, rx_mask);

	vizzini_set_reg(xr21v141x, block, UART_CLOCK_DIVISOR_0, (divisor >>  0) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_CLOCK_DIVISOR_1, (divisor >>  8) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_CLOCK_DIVISOR_2, (divisor >> 16) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_TX_CLOCK_MASK_0, (tx_mask >>  0) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_TX_CLOCK_MASK_1, (tx_mask >>  8) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_RX_CLOCK_MASK_0, (rx_mask >>  0) & 0xff);
	vizzini_set_reg(xr21v141x, block, UART_RX_CLOCK_MASK_1, (rx_mask >>  8) & 0xff);
	
	return -EINVAL;
}


static void xr21v141x_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct xr21v141x *xr21v141x = tty->driver_data;
	unsigned int             cflag, block;
        speed_t                  rate;
	unsigned int             format_size, format_parity, format_stop, flow, gpio_mode;

	cflag = tty->termios.c_cflag;

        xr21v141x->clocal = ((cflag & CLOCAL) != 0);

        block = xr21v141x->block;

        vizzini_disable(xr21v141x);

        if ((cflag & CSIZE) == CS7) {
                format_size = UART_FORMAT_SIZE_7;
        } else if ((cflag & CSIZE) == CS5) {
                /* Enabling 5-bit mode is really 9-bit mode! */
                format_size = UART_FORMAT_SIZE_9;
        } else {
                format_size = UART_FORMAT_SIZE_8;
        }
        xr21v141x->trans9 = (format_size == UART_FORMAT_SIZE_9);

        if (cflag & PARENB) {
                if (cflag & PARODD) {
                        if (cflag & CMSPAR) {
                                format_parity = UART_FORMAT_PARITY_1;
                        } else {
                                format_parity = UART_FORMAT_PARITY_ODD;
                        }
                } else {
                        if (cflag & CMSPAR) {
                                format_parity = UART_FORMAT_PARITY_0;
                        } else {
                                format_parity = UART_FORMAT_PARITY_EVEN;
                        }
                }
        } else {
                format_parity = UART_FORMAT_PARITY_NONE;
        }

        if (cflag & CSTOPB) {
                format_stop = UART_FORMAT_STOP_2;
        } else {
                format_stop = UART_FORMAT_STOP_1;
        }

#ifdef VIZZINI_IWA
        if (format_size == UART_FORMAT_SIZE_8) {
                xr21v141x->iwa = format_parity;
                if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                        format_size = UART_FORMAT_SIZE_9;
                        format_parity = UART_FORMAT_PARITY_NONE;
                }
        } else {
                xr21v141x->iwa = UART_FORMAT_PARITY_NONE;
        }
#endif
        vizzini_set_reg(xr21v141x, block, UART_FORMAT, format_size | format_parity | format_stop);

        if (cflag & CRTSCTS) {
                flow      = UART_FLOW_MODE_HW;
                gpio_mode = UART_GPIO_MODE_SEL_RTS_CTS;
        } else if (I_IXOFF(tty) || I_IXON(tty)) {
                unsigned char   start_char = START_CHAR(tty);
                unsigned char   stop_char  = STOP_CHAR(tty);

                flow      = UART_FLOW_MODE_SW;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;

                vizzini_set_reg(xr21v141x, block, UART_XON_CHAR, start_char);
                vizzini_set_reg(xr21v141x, block, UART_XOFF_CHAR, stop_char);
        } else {
                flow      = UART_FLOW_MODE_NONE;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;
        }

        vizzini_set_reg(xr21v141x, block, UART_FLOW, flow);
        vizzini_set_reg(xr21v141x, block, UART_GPIO_MODE, gpio_mode);

        if (xr21v141x->trans9) {
                /* Turn on wide mode if we're 9-bit transparent. */
                vizzini_set_reg(xr21v141x, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 1);
#ifdef VIZZINI_IWA
        } else if (xr21v141x->iwa != UART_FORMAT_PARITY_NONE) {
                vizzini_set_reg(xr21v141x, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 1);
#endif
        } else if (!xr21v141x->preciseflags) {
                /* Turn off wide mode unless we have precise flags. */
                vizzini_set_reg(xr21v141x, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 0);
        }

        rate = cpu_to_le32(tty_get_baud_rate(tty));
	if(rate)
		vizzini_set_baud_rate(xr21v141x, rate);

        xr21v141x->line.dwDTERate   = rate;
	xr21v141x->line.bCharFormat = format_stop;
        xr21v141x->line.bParityType = format_parity;
        xr21v141x->line.bDataBits   = format_size;

        vizzini_enable(xr21v141x);
}

static const struct tty_port_operations xr21v141x_port_ops = {
	.shutdown = xr21v141x_port_shutdown,
	.activate = xr21v141x_port_activate,
	.destruct = xr21v141x_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void xr21v141x_write_buffers_free(struct xr21v141x *xr21v141x)
{
	int i;
	struct xr21v141x_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(xr21v141x->control);

	for (wb = &xr21v141x->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(usb_dev, xr21v141x->writesize, wb->buf, wb->dmah);
}

static void xr21v141x_read_buffers_free(struct xr21v141x *xr21v141x)
{
	struct usb_device *usb_dev = interface_to_usbdev(xr21v141x->control);
	int i;

	for (i = 0; i < xr21v141x->rx_buflimit; i++)
		usb_free_coherent(usb_dev, xr21v141x->readsize,
			  xr21v141x->read_buffers[i].base, xr21v141x->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int xr21v141x_write_buffers_alloc(struct xr21v141x *xr21v141x)
{
	int i;
	struct xr21v141x_wb *wb;

	for (wb = &xr21v141x->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(xr21v141x->dev, xr21v141x->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(xr21v141x->dev, xr21v141x->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int xr21v141x_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct xr21v141x *xr21v141x;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int call_interface_num = -1;
	int data_interface_num = -1;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	int combined_interfaces = 0;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;
	num_rx_buf = ACM_NR;

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (union_header) {
				dev_err(&intf->dev, "More than one "
					"union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			ac_management_function = buffer[3];
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			call_management_function = buffer[3];
			call_interface_num = buffer[4];
			if ((quirks & NOT_A_MODEM) == 0 && (call_management_function & 3) != 3)
				dev_err(&intf->dev, "This device cannot do calls on its own. It is not a modem.\n");
			break;
		default:
			/* there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: "
					"type %02x, length %d\n",
					buffer[2], buffer[0]);
			break;
		}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (!union_header) {
		if (call_interface_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			/* quirks for Droids MuIn LCD */
			if (quirks & NO_DATA_INTERFACE)
				data_interface = usb_ifnum_to_if(usb_dev, 0);
			else
				data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
			control_interface = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev,"No union descriptor, giving up\n");
				return -ENODEV;
			} else {
				dev_warn(&intf->dev,"No union descriptor, testing for castrated device\n");
				combined_interfaces = 1;
				control_interface = data_interface = intf;
				goto look_for_collapsed_interface;
			}
		}
	} else {
		control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
		if (!control_interface || !data_interface) {
			dev_dbg(&intf->dev, "no interfaces\n");
			return -ENODEV;
		}
	}

	if (data_interface_num != call_interface_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	if (control_interface == data_interface) {
		/* some broken devices designed for windows work this way */
		dev_warn(&intf->dev,"Control and data interfaces are not separated!\n");
		combined_interfaces = 1;
		/* a popular other OS doesn't use it */
		quirks |= NO_CAP_LINE;
		if (data_interface->cur_altsetting->desc.bNumEndpoints != 3) {
			dev_err(&intf->dev, "This needs exactly 3 endpoints\n");
			return -EINVAL;
		}
look_for_collapsed_interface:
		for (i = 0; i < 3; i++) {
			struct usb_endpoint_descriptor *ep;
			ep = &data_interface->cur_altsetting->endpoint[i].desc;

			if (usb_endpoint_is_int_in(ep))
				epctrl = ep;
			else if (usb_endpoint_is_bulk_out(ep))
				epwrite = ep;
			else if (usb_endpoint_is_bulk_in(ep))
				epread = ep;
			else
				return -EINVAL;
		}
		if (!epctrl || !epread || !epwrite)
			return -ENODEV;
		else
			goto made_compressed_probe;
	}

	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass
						!= CDC_DATA_INTERFACE_TYPE) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass
						== CDC_DATA_INTERFACE_TYPE) {
			struct usb_interface *t;
			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			t = control_interface;
			control_interface = data_interface;
			data_interface = t;
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		struct usb_endpoint_descriptor *t;
		//dev_dbg(&intf->dev,
		//	"The data interface has switched endpoints\n");
		t = epread;
		epread = epwrite;
		epwrite = t;
	}
made_compressed_probe:
	dev_dbg(&intf->dev, "interfaces are valid\n");

	xr21v141x = kzalloc(sizeof(struct xr21v141x), GFP_KERNEL);
	if (xr21v141x == NULL) {
		dev_err(&intf->dev, "out of memory (xr21v141x kzalloc)\n");
		goto alloc_fail;
	}

	minor = xr21v141x_alloc_minor(xr21v141x);
	if (minor == XR21V141X_TTY_MINORS) {
		dev_err(&intf->dev, "no more free xr21v141x devices\n");
		kfree(xr21v141x);
		return -ENODEV;
	}

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) *
				(quirks == SINGLE_RX_URB ? 1 : 2);
	xr21v141x->combined_interfaces = combined_interfaces;
	xr21v141x->writesize = usb_endpoint_maxp(epwrite) * 20;
	xr21v141x->control = control_interface;
	xr21v141x->data = data_interface;
	xr21v141x->minor = minor;
	xr21v141x->dev = usb_dev;
	xr21v141x->ctrl_caps = ac_management_function;
	if (quirks & NO_CAP_LINE)
		xr21v141x->ctrl_caps &= ~USB_CDC_CAP_LINE;
	xr21v141x->ctrlsize = ctrlsize;
	xr21v141x->readsize = readsize;
	xr21v141x->rx_buflimit = num_rx_buf;
	INIT_WORK(&xr21v141x->work, xr21v141x_softint);
	spin_lock_init(&xr21v141x->write_lock);
	spin_lock_init(&xr21v141x->read_lock);
	mutex_init(&xr21v141x->mutex);
	xr21v141x->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	xr21v141x->is_int_ep = usb_endpoint_xfer_int(epread);
	if (xr21v141x->is_int_ep)
		xr21v141x->bInterval = epread->bInterval;
	tty_port_init(&xr21v141x->port);
	xr21v141x->port.ops = &xr21v141x_port_ops;

        xr21v141x->block = epwrite->bEndpointAddress - 1;

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &xr21v141x->ctrl_dma);
	if (!buf) {
		dev_err(&intf->dev, "out of memory (ctrl buffer alloc)\n");
		goto alloc_fail2;
	}
	xr21v141x->ctrl_buffer = buf;

	if (xr21v141x_write_buffers_alloc(xr21v141x) < 0) {
		dev_err(&intf->dev, "out of memory (write buffer alloc)\n");
		goto alloc_fail4;
	}

	xr21v141x->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!xr21v141x->ctrlurb) {
		dev_err(&intf->dev, "out of memory (ctrlurb kmalloc)\n");
		goto alloc_fail5;
	}
	for (i = 0; i < num_rx_buf; i++) {
		struct xr21v141x_rb *rb = &(xr21v141x->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(xr21v141x->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base) {
			dev_err(&intf->dev, "out of memory "
					"(read bufs usb_alloc_coherent)\n");
			goto alloc_fail6;
		}
		rb->index = i;
		rb->instance = xr21v141x;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			dev_err(&intf->dev,
				"out of memory (read urbs usb_alloc_urb)\n");
			goto alloc_fail6;
		}
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (xr21v141x->is_int_ep) {
			usb_fill_int_urb(urb, xr21v141x->dev,
					 xr21v141x->rx_endpoint,
					 rb->base,
					 xr21v141x->readsize,
					 xr21v141x_read_bulk_callback, rb,
					 xr21v141x->bInterval);
		} else {
			usb_fill_bulk_urb(urb, xr21v141x->dev,
					  xr21v141x->rx_endpoint,
					  rb->base,
					  xr21v141x->readsize,
					  xr21v141x_read_bulk_callback, rb);
		}

		xr21v141x->read_urbs[i] = urb;
		__set_bit(i, &xr21v141x->read_urbs_free);
	}
	for (i = 0; i < ACM_NW; i++) {
		struct xr21v141x_wb *snd = &(xr21v141x->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL) {
			dev_err(&intf->dev,
				"out of memory (write urbs usb_alloc_urb)\n");
			goto alloc_fail7;
		}

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev,
				usb_sndintpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, xr21v141x->writesize, xr21v141x_write_bulk, snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, xr21v141x->writesize, xr21v141x_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = xr21v141x;
	}

	usb_set_intfdata(intf, xr21v141x);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail7;

	if (cfd) { /* export the country data */
		xr21v141x->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!xr21v141x->country_codes)
			goto skip_countries;
		xr21v141x->country_code_size = cfd->bLength - 4;
		memcpy(xr21v141x->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		xr21v141x->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(xr21v141x->country_codes);
			xr21v141x->country_codes = NULL;
			xr21v141x->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(xr21v141x->country_codes);
			xr21v141x->country_codes = NULL;
			xr21v141x->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(xr21v141x->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 xr21v141x->ctrl_buffer, ctrlsize, xr21v141x_ctrl_irq, xr21v141x,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 0xff);
	xr21v141x->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	xr21v141x->ctrlurb->transfer_dma = xr21v141x->ctrl_dma;

	dev_info(&intf->dev, "ttyUSB%d: XR21v14x usb uart device\n", minor);

	xr21v141x_set_control(xr21v141x, xr21v141x->ctrlout);

	xr21v141x->line.dwDTERate = cpu_to_le32(9600);
	xr21v141x->line.bDataBits = 8;
	xr21v141x_set_line(xr21v141x, &xr21v141x->line);

	usb_driver_claim_interface(&xr21v141x_driver, data_interface, xr21v141x);
	usb_set_intfdata(data_interface, xr21v141x);

	usb_get_intf(control_interface);
	tty_register_device(xr21v141x_tty_driver, minor, &control_interface->dev);

	return 0;
alloc_fail7:
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(xr21v141x->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(xr21v141x->read_urbs[i]);
	xr21v141x_read_buffers_free(xr21v141x);
	usb_free_urb(xr21v141x->ctrlurb);
alloc_fail5:
	xr21v141x_write_buffers_free(xr21v141x);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, xr21v141x->ctrl_buffer, xr21v141x->ctrl_dma);
alloc_fail2:
	xr21v141x_release_minor(xr21v141x);
	kfree(xr21v141x);
alloc_fail:
	return -ENOMEM;
}

static void stop_data_traffic(struct xr21v141x *xr21v141x)
{
	int i;

	dev_dbg(&xr21v141x->control->dev, "%s\n", __func__);

	usb_kill_urb(xr21v141x->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(xr21v141x->wb[i].urb);
	for (i = 0; i < xr21v141x->rx_buflimit; i++)
		usb_kill_urb(xr21v141x->read_urbs[i]);

	cancel_work_sync(&xr21v141x->work);
}

static void xr21v141x_disconnect(struct usb_interface *intf)
{
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i;

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!xr21v141x)
		return;

	mutex_lock(&xr21v141x->mutex);
	xr21v141x->disconnected = true;
	if (xr21v141x->country_codes) {
		device_remove_file(&xr21v141x->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&xr21v141x->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	device_remove_file(&xr21v141x->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(xr21v141x->control, NULL);
	usb_set_intfdata(xr21v141x->data, NULL);
	mutex_unlock(&xr21v141x->mutex);

	tty = tty_port_tty_get(&xr21v141x->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	stop_data_traffic(xr21v141x);

	usb_free_urb(xr21v141x->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(xr21v141x->wb[i].urb);
	for (i = 0; i < xr21v141x->rx_buflimit; i++)
		usb_free_urb(xr21v141x->read_urbs[i]);
	xr21v141x_write_buffers_free(xr21v141x);
	usb_free_coherent(usb_dev, xr21v141x->ctrlsize, xr21v141x->ctrl_buffer, xr21v141x->ctrl_dma);
	xr21v141x_read_buffers_free(xr21v141x);

	if (!xr21v141x->combined_interfaces)
		usb_driver_release_interface(&xr21v141x_driver, intf == xr21v141x->control ?
					xr21v141x->data : xr21v141x->control);

	tty_port_put(&xr21v141x->port);
}

#ifdef CONFIG_PM
static int xr21v141x_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);
	int cnt;

	if (PMSG_IS_AUTO(message)) {
		int b;

		spin_lock_irq(&xr21v141x->write_lock);
		b = xr21v141x->transmitting;
		spin_unlock_irq(&xr21v141x->write_lock);
		if (b)
			return -EBUSY;
	}

	spin_lock_irq(&xr21v141x->read_lock);
	spin_lock(&xr21v141x->write_lock);
	cnt = xr21v141x->susp_count++;
	spin_unlock(&xr21v141x->write_lock);
	spin_unlock_irq(&xr21v141x->read_lock);

	if (cnt)
		return 0;

	if (test_bit(ASYNCB_INITIALIZED, &xr21v141x->port.flags))
		stop_data_traffic(xr21v141x);

	return 0;
}

static int xr21v141x_resume(struct usb_interface *intf)
{
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);
	struct xr21v141x_wb *wb;
	int rv = 0;
	int cnt;

	spin_lock_irq(&xr21v141x->read_lock);
	xr21v141x->susp_count -= 1;
	cnt = xr21v141x->susp_count;
	spin_unlock_irq(&xr21v141x->read_lock);

	if (cnt)
		return 0;

	if (test_bit(ASYNCB_INITIALIZED, &xr21v141x->port.flags)) {
		rv = usb_submit_urb(xr21v141x->ctrlurb, GFP_NOIO);

		spin_lock_irq(&xr21v141x->write_lock);
		if (xr21v141x->delayed_wb) {
			wb = xr21v141x->delayed_wb;
			xr21v141x->delayed_wb = NULL;
			spin_unlock_irq(&xr21v141x->write_lock);
			xr21v141x_start_wb(xr21v141x, wb);
		} else {
			spin_unlock_irq(&xr21v141x->write_lock);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto err_out;

		rv = xr21v141x_submit_read_urbs(xr21v141x, GFP_NOIO);
	}

err_out:
	return rv;
}

static int xr21v141x_reset_resume(struct usb_interface *intf)
{
	struct xr21v141x *xr21v141x = usb_get_intfdata(intf);
	struct tty_struct *tty;

	if (test_bit(ASYNCB_INITIALIZED, &xr21v141x->port.flags)) {
		tty = tty_port_tty_get(&xr21v141x->port);
		if (tty) {
			tty_hangup(tty);
			tty_kref_put(tty);
		}
	}

	return xr21v141x_resume(intf);
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */

static const struct usb_device_id xrusb_ids[] = {
	{ USB_DEVICE(0x04e2, 0x1410),
	},
	{ USB_DEVICE(0x04e2, 0x1412),
	},
	{ USB_DEVICE(0x04e2, 0x1414),
	},
	{ }
};

MODULE_DEVICE_TABLE(usb, xrusb_ids);

static struct usb_driver xr21v141x_driver = {
	.name =		"vizzini",
	.probe =	xr21v141x_probe,
	.disconnect =	xr21v141x_disconnect,
#ifdef CONFIG_PM
	.suspend =	xr21v141x_suspend,
	.resume =	xr21v141x_resume,
	.reset_resume =	xr21v141x_reset_resume,
#endif
	.id_table =	xrusb_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

/*
 * TTY driver structures.
 */

static const struct tty_operations xr21v141x_ops = {
	.install =		xr21v141x_tty_install,
	.open =			xr21v141x_tty_open,
	.close =		xr21v141x_tty_close,
	.cleanup =		xr21v141x_tty_cleanup,
	.hangup =		xr21v141x_tty_hangup,
	.write =		xr21v141x_tty_write,
	.write_room =		xr21v141x_tty_write_room,
	.ioctl =		xr21v141x_tty_ioctl,
	.throttle =		xr21v141x_tty_throttle,
	.unthrottle =		xr21v141x_tty_unthrottle,
	.chars_in_buffer =	xr21v141x_tty_chars_in_buffer,
	.break_ctl =		xr21v141x_tty_break_ctl,
	.set_termios =		xr21v141x_tty_set_termios,
	.tiocmget =		xr21v141x_tty_tiocmget,
	.tiocmset =		xr21v141x_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init xr21v141x_init(void)
{
	int retval;
	xr21v141x_tty_driver = alloc_tty_driver(XR21V141X_TTY_MINORS);
	if (!xr21v141x_tty_driver)
		return -ENOMEM;
	xr21v141x_tty_driver->driver_name = "vizzini",
	xr21v141x_tty_driver->name = "ttyUSB",
	xr21v141x_tty_driver->major = XR21V141X_TTY_MAJOR,
	xr21v141x_tty_driver->minor_start = 0,
	xr21v141x_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	xr21v141x_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	xr21v141x_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	xr21v141x_tty_driver->init_termios = tty_std_termios;
	xr21v141x_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;
	tty_set_operations(xr21v141x_tty_driver, &xr21v141x_ops);

	retval = tty_register_driver(xr21v141x_tty_driver);
	if (retval) {
		put_tty_driver(xr21v141x_tty_driver);
		return retval;
	}

	retval = usb_register(&xr21v141x_driver);
	if (retval) {
		tty_unregister_driver(xr21v141x_tty_driver);
		put_tty_driver(xr21v141x_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit xr21v141x_exit(void)
{
	usb_deregister(&xr21v141x_driver);
	tty_unregister_driver(xr21v141x_tty_driver);
	put_tty_driver(xr21v141x_tty_driver);
}

module_init(xr21v141x_init);
module_exit(xr21v141x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ACM_TTY_MAJOR);
