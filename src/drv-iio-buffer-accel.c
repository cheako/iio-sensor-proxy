/*
 * Copyright (c) 2014 Bastien Nocera <hadess@hadess.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 */

#include "drivers.h"
#include "iio-buffer-utils.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

/* 1G (9.81m/s²) corresponds to "256"
 * value x scale is in m/s² */
#define SCALE_TO_FF(scale) (scale * 256 / 9.81)

typedef struct {
	GIOChannel        *gio_r;
	gulong             io_source;
	ReadingsUpdateFunc callback_func;
	gpointer           user_data;

	GUdevDevice *dev;
	const char *dev_path;
	int device_id;
	BufferDrvData *buffer_data;
} DrvData;

static DrvData *drv_data = NULL;

static int
process_scan (IIOSensorData data, DrvData *or_data)
{
	int accel_x, accel_y, accel_z;
	gdouble scale;
	gboolean present_x, present_y, present_z;
	AccelReadings readings;

	process_scan_1(data.data, or_data->buffer_data, "in_accel_x", &accel_x, &scale, &present_x);
	process_scan_1(data.data, or_data->buffer_data, "in_accel_y", &accel_y, &scale, &present_y);
	process_scan_1(data.data, or_data->buffer_data, "in_accel_z", &accel_z, &scale, &present_z);

	g_debug ("Read from IIO: %d, %d, %d", accel_x, accel_y, accel_z);

	/* To match the Pegatron accelerometer code
	 * (see pega_accel_poll() in asus-laptop.c)
	 * we invert both x, and y values */
	accel_x = -accel_x;
	accel_y = -accel_y;

	//FIXME report errors
	readings.accel_x = accel_x * scale;
	readings.accel_y = accel_y * scale;
	readings.accel_z = accel_z * scale;
	or_data->callback_func (&iio_buffer_accel, (gpointer) &readings, or_data->user_data);

	return 1;
}

static gboolean
read_orientation (GIOChannel  * channel,
                  GIOCondition  condition,
                  gpointer      user_data)
{
	DrvData      *or_data       = user_data;

	GIOStatus     status;
	GError       *gerr          = NULL;

	IIOSensorData buff;
	IIOSensorData data;

	gsize         buf_size      = or_data->buffer_data->scan_size;
	static gsize  read_size_ctr = 0;

	g_assert_cmpuint (condition, ==, G_IO_IN);

	buff.data = g_malloc(buf_size);
	data.data = NULL;

	for (status = g_io_channel_read_chars (channel, buff.data, buf_size - read_size_ctr, (gsize*)&buff.read_size, &gerr);
		status == G_IO_STATUS_NORMAL;
		status = g_io_channel_read_chars (channel, buff.data, buf_size - read_size_ctr, (gsize*)&buff.read_size, &gerr))
	{
		read_size_ctr += buff.read_size;
		if(read_size_ctr >= buf_size) read_size_ctr -= buff.read_size;
		if(buff.read_size >= buf_size) {
			if(data.data) g_free (data.data);
			data.data = buff.data;
			buff.data = g_malloc(buf_size);
			data.read_size = buff.read_size;
		}
	}

	g_assert_cmpuint (status, ==, G_IO_STATUS_AGAIN);

        if(data.data) {
		process_scan(data, or_data);
		g_free (data.data);
	}

        g_free (buff.data);

	return G_SOURCE_CONTINUE;
}

static char *
get_trigger_name (GUdevDevice *device)
{
	GList *devices, *l;
	GUdevClient *client;
	gboolean has_trigger = FALSE;
	char *trigger_name;
	const gchar * const subsystems[] = { "iio", NULL };

	client = g_udev_client_new (subsystems);
	devices = g_udev_client_query_by_subsystem (client, "iio");

	/* Find the associated trigger */
	trigger_name = g_strdup_printf ("accel_3d-dev%s", g_udev_device_get_number (device));
	for (l = devices; l != NULL; l = l->next) {
		GUdevDevice *dev = l->data;

		if (g_strcmp0 (trigger_name, g_udev_device_get_sysfs_attr (dev, "name")) == 0) {
			g_debug ("Found associated trigger at %s", g_udev_device_get_sysfs_path (dev));
			has_trigger = TRUE;
			break;
		}
	}

	g_list_free_full (devices, g_object_unref);
	g_clear_object (&client);

	if (has_trigger)
		return trigger_name;

	g_warning ("Could not find trigger name associated with %s",
		   g_udev_device_get_sysfs_path (device));
	g_free (trigger_name);
	return NULL;
}

static gboolean
iio_buffer_accel_discover (GUdevDevice *device)
{
	if (g_strcmp0 (g_udev_device_get_subsystem (device), "iio") != 0)
		return FALSE;

	if (g_strcmp0 ("accel_3d", g_udev_device_get_sysfs_attr (device, "name")) != 0)
		return FALSE;

	g_debug ("Found accel_3d at %s", g_udev_device_get_sysfs_path (device));
	return TRUE;
}

static void
iio_buffer_accel_set_polling (gboolean state)
{
	if (drv_data->io_source > 0 && state)
		return;
	if (drv_data->io_source == 0 && !state)
		return;

	if (drv_data->io_source) {
		g_source_remove (drv_data->io_source);
		drv_data->io_source = 0;
	}

	if (state) {
		drv_data->io_source = g_io_add_watch (drv_data->gio_r, G_IO_IN, read_orientation, drv_data);
		g_source_set_name_by_id (drv_data->io_source, "[iio_buffer_accel_set_nonblocking] read_orientation");
	}
}

static gboolean
iio_buffer_accel_open (GUdevDevice        *device,
		       ReadingsUpdateFunc  callback_func,
		       gpointer            user_data)
{
	char *trigger_name;
	GError      *gerr = NULL;
	GIOStatus    status;

	drv_data = g_new0 (DrvData, 1);

	/* Get the trigger name, and build the channels from that */
	trigger_name = get_trigger_name (device);
	if (!trigger_name) {
		g_clear_pointer (&drv_data, g_free);
		return FALSE;
	}
	drv_data->buffer_data = buffer_drv_data_new (device, trigger_name);
	g_free (trigger_name);

	if (!drv_data->buffer_data) {
		g_clear_pointer (&drv_data, g_free);
		return FALSE;
	}

	drv_data->dev = g_object_ref (device);
	drv_data->dev_path = g_udev_device_get_device_file (device);

	drv_data->callback_func = callback_func;
	drv_data->user_data = user_data;

        drv_data->gio_r = g_io_channel_new_file (drv_data->dev_path, "r", &gerr);
	if (gerr) {
		g_error ("Unable to open file %s: %s", drv_data->dev_path, gerr->message);
		g_error_free(gerr);
		g_clear_pointer (&drv_data->buffer_data, buffer_drv_data_free);
		g_clear_object (&drv_data->dev);
		g_clear_pointer (&drv_data, g_free);
		return FALSE;
	}
	g_io_channel_set_encoding (drv_data->gio_r, NULL, &gerr);
	if (gerr) {
		g_error ("g_io_channel_set_encoding: %s: %s", drv_data->dev_path, gerr->message);
		g_error_free (gerr);
		g_clear_pointer (&drv_data->buffer_data, buffer_drv_data_free);
		g_io_channel_unref(drv_data->gio_r);
		g_clear_object (&drv_data->dev);
		g_clear_pointer (&drv_data, g_free);
		return FALSE;
	}
	g_io_channel_set_buffer_size (drv_data->gio_r, 4096);

	status = g_io_channel_set_flags (drv_data->gio_r, G_IO_FLAG_NONBLOCK, &gerr);
	if (status == G_IO_STATUS_ERROR) {
		g_warning ("g_io_channel_set_flags: %s: %s", drv_data->dev_path, gerr->message);
		g_error_free (gerr);
		gerr = NULL;
	}

	return TRUE;
}

static void
iio_buffer_accel_close (void)
{
	iio_buffer_accel_set_polling (FALSE);
	g_clear_pointer (&drv_data->buffer_data, buffer_drv_data_free);
	g_io_channel_unref(drv_data->gio_r);
	g_clear_object (&drv_data->dev);
	g_clear_pointer (&drv_data, g_free);
}

SensorDriver iio_buffer_accel = {
	.name = "IIO Buffer accelerometer",
	.type = DRIVER_TYPE_ACCEL,
	.specific_type = DRIVER_TYPE_ACCEL_IIO,

	.discover = iio_buffer_accel_discover,
	.open = iio_buffer_accel_open,
	.set_polling = iio_buffer_accel_set_polling,
	.close = iio_buffer_accel_close,
};
