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

	printk("Device name is %s\n", dev->name);

	struct sensor_value val, fid;
	int ret;

	fid.val1 = 1;
	fid.val2 = 1;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_FINGERPRINT, &val);
	printk("template count : %d\n", val.val1);
	printk("Capacity of FPS : %d\n", val.val2);

	ret = sensor_attr_set(dev, SENSOR_CHAN_FINGERPRINT, SENSOR_ATTR_ADD, &fid);
	if (ret != 0) {
		printk("Sensor attr set failed %d\n", ret);
	}


}
