/*
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT grow_r502a

#include <arch/cpu.h>
#include <init.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/uart.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>

#include "grow_r502a.h"

LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

const struct sys_params defaults = {
	.status_reg = 0x0,
	.system_id = 0x0,
	.capacity = 200,
	.security_level = 5,
	.device_addr = 0xFFFFFFFF,
	.packet_len = 64,
	.baud_rate = 57600,
};

static inline void create_packet(struct packet *packet, uint8_t pkg_type,
		uint8_t *pkg_data, uint16_t len)
{
	packet->start_code = FPS_STARTCODE;
	packet->type = pkg_type;
	packet->length = len;
	packet->address[0] = 0xFF;
	packet->address[1] = 0xFF;
	packet->address[2] = 0xFF;
	packet->address[3] = 0xFF;
	if (len < 64) {
		memcpy(packet->data, pkg_data, len);
	} else {
		memcpy(packet->data, pkg_data, 64);
	}
}

static void write_packet(const struct device *dev, struct packet gen_packet)
{
	uint16_t length = gen_packet.length + 2;

	uint8_t packet[] = { (gen_packet.start_code >> 8), gen_packet.start_code & 0xFF,
					    gen_packet.address[0], gen_packet.address[1],
					    gen_packet.address[2], gen_packet.address[3],
					    gen_packet.type,
						(length >> 8), (length & 0xFF) };

	for (uint8_t i = 0; i < sizeof(packet); i++) {
		uart_poll_out(dev, packet[i]);
	}
	uint16_t sum = (length >> 8) + (length & 0xFF) + gen_packet.type;

	for (uint8_t i = 0; i < gen_packet.length; i++) {
		uart_poll_out(dev, gen_packet.data[i]);
		sum += gen_packet.data[i];
	}

	uint8_t sum_pkt[] = { (sum >> 8), (sum & 0xFF)};

	uart_poll_out(dev, sum_pkt[0]);
	uart_poll_out(dev, sum_pkt[1]);
}

static uint8_t fps_get_image(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_GENIMAGE};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	return drv_data->recv_packet.data[0];
}

static int16_t fps_image2Tz(const struct device *dev, uint8_t slot)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_IMAGE2TZ, slot};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	return drv_data->recv_packet.data[0];
}

static int16_t fps_create_model(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_REGMODEL};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	return drv_data->recv_packet.data[0];
}

static int16_t fps_store_model(const struct device *dev, uint16_t id, uint8_t slot)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_STORE, slot, id >> 8, id & 0xFF};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	return drv_data->recv_packet.data[0];
}

static int16_t fps_get_template_count(const struct device *dev, uint16_t *template_cnt)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_TEMPLATECOUNT};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	*template_cnt = drv_data->recv_packet.data[1];
	*template_cnt <<= 8;
	*template_cnt |= drv_data->recv_packet.data[2];

	return drv_data->recv_packet.data[0];
}

static void get_sys_parameter(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	struct packet gen_packet;
	uint8_t data[] = {FPS_READSYSPARAM};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(dev, gen_packet);

	drv_data->params.ack_byte = drv_data->recv_packet.data[0];
	drv_data->params.status_reg = ((uint16_t)drv_data->recv_packet.data[1] << 8) |
		drv_data->recv_packet.data[2];
	drv_data->params.system_id = ((uint16_t)drv_data->recv_packet.data[3] << 8) |
		drv_data->recv_packet.data[4];
	drv_data->params.capacity = ((uint16_t)drv_data->recv_packet.data[5] << 8) |
		drv_data->recv_packet.data[6];
	drv_data->params.security_level = ((uint16_t)drv_data->recv_packet.data[7] << 8) |
		drv_data->recv_packet.data[8];
	drv_data->params.device_addr = ((uint32_t)drv_data->recv_packet.data[9] << 24) |
		((uint32_t)drv_data->recv_packet.data[10] << 16) |
		((uint32_t)drv_data->recv_packet.data[11] << 8) |
		(uint32_t)drv_data->recv_packet.data[12];
	drv_data->params.packet_len = ((uint16_t)drv_data->recv_packet.data[13] << 8) |
		drv_data->recv_packet.data[14];
	if (drv_data->params.packet_len == 0) {
		drv_data->params.packet_len = 32;
	} else if (drv_data->params.packet_len == 1) {
		drv_data->params.packet_len = 64;
	} else if (drv_data->params.packet_len == 2) {
		drv_data->params.packet_len = 128;
	} else if (drv_data->params.packet_len == 3) {
		drv_data->params.packet_len = 256;
	}
	drv_data->params.baud_rate = (((uint16_t)drv_data->recv_packet.data[15] << 8) |
			drv_data->recv_packet.data[16]) * 9600;

}

static int grow_r502a_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{
	struct grow_r502a_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		fps_get_template_count(dev, &drv_data->count);
		get_sys_parameter(dev);
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static int grow_r502a_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_FINGERPRINT) {
			val->val1 = drv_data->count;
			LOG_INF("Get template count OK\n");
			val->val2 = drv_data->params.capacity;
			LOG_INF("Get capacity of FPS OK\n");
		} else {
			LOG_ERR("Sensor channel_get error\n");
			return -EINVAL;
		}

		return 0;
}

static int grow_r502a_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		if (attr == SENSOR_ATTR_ADD) {
			int16_t p = -1;

			LOG_INF("Waiting for valid finger to enroll as ID #%d", val->val1);

			while (p != FPS_OK) {
				p = fps_get_image(dev);
				switch (p) {
				case FPS_OK:
				      LOG_INF("Image taken");
				break;
				case FPS_NOFINGER:
				      LOG_INF(".");
				break;
				default:
				      LOG_ERR("error code 0x%X", p);
				break;
				}
			}

			p = fps_image2Tz(dev, val->val2);
			switch (p) {
			case FPS_OK:
			      LOG_INF("Image converted");
			break;
			default:
			      LOG_ERR("error code 0x%X", p);
			break;
			}
			LOG_INF("Remove finger");
			k_msleep(2000);
			p = 0;
			while (p != FPS_NOFINGER) {
				p = fps_get_image(dev);
				k_msleep(10);
			}
			LOG_INF("ID %d", val->val1);
			p = -1;
			LOG_INF("Place same finger again");
			while (p != FPS_OK) {
				p = fps_get_image(dev);
				switch (p) {
				case FPS_OK:
					LOG_INF("Image taken");
					break;
				default:
					LOG_ERR("error code 0x%X", p);
					break;
				}
			}

			p = fps_image2Tz(dev, val->val2+1);
			switch (p) {
			case FPS_OK:
				LOG_INF("Image converted");
				break;
			default:
			      LOG_ERR("error code 0x%X", p);
				break;
			}

			LOG_INF("Creating model for #%d", val->val1);

			p = fps_create_model(dev);
			if (p == FPS_OK) {
				LOG_INF("Prints matched!");
			} else {
				LOG_ERR("error code 0x%X", p);
			}

			LOG_INF("ID %d", val->val1);
			p = fps_store_model(dev, val->val1, val->val2);
			if (p == FPS_OK) {
				LOG_INF("Stored!");
			} else {
				LOG_ERR("error code 0x%X", p);
			}
		} else {
			return -ENOTSUP;
		}
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int grow_r502a_attr_get(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    struct sensor_value *val)
{

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		if (attr) { /*To Do*/
			/*To Do*/
		} else {
			return -ENOTSUP;
		}
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static void work_handler(struct k_work *work)
{
	struct grow_r502a_data *drv_data =
		CONTAINER_OF(work, struct grow_r502a_data, work);
	uint8_t buffer[64];
	uint16_t idx = 0;

	ring_buf_get(&drv_data->ringbuf, buffer, sizeof(buffer));

	while (true) {
		switch (idx) {
		case 0:
			if (buffer[idx] != (FPS_STARTCODE >> 8)) {
				continue;
			}
			drv_data->recv_packet.start_code = (uint16_t)buffer[idx] << 8;
			break;
		case 1:
			drv_data->recv_packet.start_code |= buffer[idx];
			if (drv_data->recv_packet.start_code != FPS_STARTCODE) {
				LOG_ERR("Received Bad Packet %X\n", FPS_BADPACKET);
			}
			break;
		case 2:
			drv_data->recv_packet.address[idx-2] = buffer[idx];
			break;
		case 3:
			drv_data->recv_packet.address[idx-2] = buffer[idx];
			break;
		case 4:
			drv_data->recv_packet.address[idx-2] = buffer[idx];
			break;
		case 5:
			drv_data->recv_packet.address[idx-2] = buffer[idx];
			break;
		case 6:
			drv_data->recv_packet.type = buffer[idx];
			break;
		case 7:
			drv_data->recv_packet.length = (uint16_t)buffer[idx] << 8;
			break;
		case 8:
			drv_data->recv_packet.length |= buffer[idx];
			break;
		default:
		{
			int buf_id = idx;

			for (uint8_t i = 0; i < (sizeof(buffer) - idx); i++, buf_id++) {
				drv_data->recv_packet.data[i] = buffer[buf_id];
			}
			if ((idx - 8) == drv_data->recv_packet.length) {
				LOG_INF("Confirmation code %X\n", FPS_OK);
			} else {
				LOG_ERR("Confirmation code %X\n", FPS_BADPACKET);
			}
			break;
		}
		}
		idx++;
	}
}

static void grow_r502a_uart_isr(const struct device *dev, void *user_data)
{
	struct grow_r502a_data *drv_data = dev->data;
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&drv_data->ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("No Data is Receiving\n");
			}

			rb_len = ring_buf_put(&drv_data->ringbuf, buffer, recv_len);
			if (rb_len > 10) {
				k_work_submit(&drv_data->work);
			} else {
				LOG_ERR("Data receive error\n");
			}
		}
	}
}

static int grow_r502a_init(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;

	drv_data->dev = device_get_binding(DT_INST_BUS_LABEL(0));

	if (!drv_data->dev) {
		LOG_DBG("uart device is not found: %s",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	ring_buf_init(&drv_data->ringbuf, sizeof(drv_data->rx_buf),
		drv_data->rx_buf);
	k_work_init(&drv_data->work, work_handler);

	uart_irq_callback_set(dev, grow_r502a_uart_isr);
	uart_irq_rx_enable(dev);

	return 0;
}

static const struct sensor_driver_api grow_r502a_api = {
	.sample_fetch = grow_r502a_sample_fetch,
	.channel_get = grow_r502a_channel_get,
	.attr_set = grow_r502a_attr_set,
	.attr_get = grow_r502a_attr_get,
};

static struct grow_r502a_data grow_r502a_data;

DEVICE_DT_INST_DEFINE(0, &grow_r502a_init, NULL,
		    &grow_r502a_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &grow_r502a_api);
