/*
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT grow_r502a

#include <drivers/sensor.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/uart.h>
#include <logging/log.h>

#include "grow_r502a.h"

LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

static int grow_r502a_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{

}

static int grow_r502a_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{

}

static int grow_r502a_trigger_set(const struct device *dev,
				 const struct sensor_trigger *trig,
				 sensor_trigger_handler_t handler)
{

}

int grow_r502a_init_interrupt(const struct device *dev)
{

}

static const struct sensor_driver_api grow_r502a_api = {
	.trigger_set = grow_r502a_trigger_set,
	.sample_fetch = &grow_r502a_sample_fetch,
	.channel_get = &grow_r502a_channel_get,
	.attr_set = grow_r502a_attr_set,
	.attr_get = grow_r502a_attr_get,
};

static int grow_r502a_init(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;

	drv_data->uart_dev = device_get_binding(DT_INST_BUS_LABEL(0));

	if (!drv_data->uart_dev) {
		LOG_DBG("uart device is not found: %s",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static struct grow_r502a_data grow_r502a_data;

DEVICE_DT_INST_DEFINE(0, &grow_r502a_init, NULL,
		    &grow_r502a_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &grow_r502a_api);
