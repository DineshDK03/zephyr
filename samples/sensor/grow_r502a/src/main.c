/*
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

void main(void)
{
	const struct device *dev =  device_get_binding(DT_LABEL(DT_INST(0, grow_r502a)));
	struct sensor_value fid;

	printk("Device name is %s\n", dev->name);

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_FINGERPRINT, &fid);
	printk("template count : %d", fid.val1);
}
