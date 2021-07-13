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
#include <sys/byteorder.h>

#include "grow_r502a.h"

LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

static const struct device *dev;

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

static void write_packet(struct packet gen_packet)
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

static uint8_t getStructuredPacket(const struct device *uart_dev, struct packet *packet,
		uint16_t timeout)
{
	uint8_t byte;
	uint16_t idx = 0;

	while (true) {
		if (sys_clock_tick_get() - sys_clock_timeout_end_calc(K_MSEC(timeout)) <= 0) {
			printk("Timeout happened\n");
		}

		uart_fifo_read(uart_dev, &byte, 1);

		switch (idx) {
		case 0:
			if (byte != (FPS_STARTCODE >> 8)) {
				continue;
			}
			packet->start_code = (uint16_t)byte << 8;
			break;
		case 1:
			packet->start_code |= byte;
			if (packet->start_code != FPS_STARTCODE) {
				return FPS_BADPACKET;
			}
			break;
		case 2:
		case 3:
		case 4:
		case 5:
			packet->address[idx - 2] = byte;
			break;
		case 6:
			packet->type = byte;
			break;
		case 7:
			packet->length = (uint16_t)byte << 8;
			break;
		case 8:
			packet->length |= byte;
			break;
		default:
			packet->data[idx - 9] = byte;
			if ((idx - 8) == packet->length) {
				return FPS_OK;
			}
			break;
		}
		idx++;
	}
	/* Shouldn't get here so... */
	return FPS_BADPACKET;
}

static int16_t fps_get_image(void)
{
	uint8_t data[] = {FPS_GENIMAGE};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	return recv_packet.data[0];
}

static int16_t fps_image2Tz(uint8_t slot)
{
	uint8_t data[] = {FPS_IMAGE2TZ, slot};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	return recv_packet.data[0];
}

static int16_t fps_create_model(void)
{
	uint8_t data[] = {FPS_REGMODEL};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	return recv_packet.data[0];
}

static int16_t fps_store_model(uint16_t id, uint8_t slot)
{
	uint8_t data[] = {FPS_STORE, slot, id >> 8, id & 0xFF};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	return recv_packet.data[0];
}

static int16_t fps_get_template_count(uint16_t *template_cnt)
{
	uint8_t data[0] = {FPS_TEMPLATECOUNT};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	*template_cnt = recv_packet.data[1];
	*template_cnt <<= 8;
	*template_cnt |= recv_packet.data[2];

	return recv_packet.data[0];
}

static struct sys_params getParameters(void)
{
	struct sys_params get_param;
	uint8_t data[] = {FPS_READSYSPARAM};

	create_packet(&gen_packet, FPS_COMMANDPACKET, data, sizeof(data));
	write_packet(gen_packet);

	get_param.return_data = recv_packet.data[0];
	get_param.status_reg = ((uint16_t)recv_packet.data[1] << 8) | recv_packet.data[2];
	get_param.system_id = ((uint16_t)recv_packet.data[3] << 8) | recv_packet.data[4];
	get_param.capacity = ((uint16_t)recv_packet.data[5] << 8) | recv_packet.data[6];
	get_param.security_level = ((uint16_t)recv_packet.data[7] << 8) | recv_packet.data[8];
	get_param.device_addr = ((uint32_t)recv_packet.data[9] << 24) |
		((uint32_t)recv_packet.data[10] << 16) |
		((uint32_t)recv_packet.data[11] << 8) | (uint32_t)recv_packet.data[12];
	get_param.packet_len = ((uint16_t)recv_packet.data[13] << 8) | recv_packet.data[14];
	if (get_param.packet_len == 0) {
		get_param.packet_len = 32;
	} else if (get_param.packet_len == 1) {
		get_param.packet_len = 64;
	} else if (get_param.packet_len == 2) {
		get_param.packet_len = 128;
	} else if (get_param.packet_len == 3) {
		get_param.packet_len = 256;
	}
	get_param.baud_rate = (((uint16_t)recv_packet.data[15] << 8) | recv_packet.data[16]) * 9600;

	return get_param;
}

static int grow_r502a_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		printk("Channel fetch success");
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
		uint8_t rc = fps_get_template_count(&drv_data->count);
		if (rc == FPS_OK) {
			LOG_INF("Get template count OK\n");
			val->val1 = drv_data->count;
		} else {
			LOG_ERR("template count Get Error : 0x%X\n", rc);
		}
		struct sys_params get_param = getParameters();

		if (get_param.return_data == FPS_OK) {
			LOG_INF("Get capacity of FPS OK\n");
			val->val2 = get_param.capacity;
		} else {
			LOG_ERR("Capacity Get Error : 0x%X\n", get_param.return_data);
		}
	} else {
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
				p = fps_get_image();
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

			p = fps_image2Tz(val->val2);
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
				p = fps_get_image();
				k_msleep(10);
			}
			LOG_INF("ID %d", val->val1);
			p = -1;
			LOG_INF("Place same finger again");
			while (p != FPS_OK) {
				p = fps_get_image();
				switch (p) {
				case FPS_OK:
					LOG_INF("Image taken");
					break;
				default:
					LOG_ERR("error code 0x%X", p);
					break;
				}
			}

			p = fps_image2Tz(val->val2+1);
			switch (p) {
			case FPS_OK:
				LOG_INF("Image converted");
				break;
			default:
			      LOG_ERR("error code 0x%X", p);
				break;
			}

			LOG_INF("Creating model for #%d", val->val1);

			p = fps_create_model();
			if (p == FPS_OK) {
				LOG_INF("Prints matched!");
			} else {
				LOG_ERR("error code 0x%X", p);
			}

			LOG_INF("ID %d", val->val1);
			p = fps_store_model(val->val1, val->val2);
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

static void grow_r502a_work_cb(struct k_work *work)
{
	struct grow_r502a_data *drv_data =
		CONTAINER_OF(work, struct grow_r502a_data, work);

	if (drv_data->touch_handler != NULL) {
		uart_irq_tx_disable(dev);
		uart_irq_rx_disable(dev);
		uart_irq_callback_set(dev, grow_r502a_uart_isr);
		uart_irq_rx_enable(dev);
	}
}

static int grow_r502a_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct grow_r502a_data *drv_data = dev->data;

	if (trig->type == SENSOR_TRIG_TOUCH) {
		drv_data->touch_handler = handler;
		drv_data->touch_trigger = *trig;
	} else {
		return -ENOTSUP;
	}

	return 0;

}

static int grow_r502a_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint16_t timeout = 1000;

	if (!uart_irq_update(dev)) {
		LOG_ERR("retval should always be 1\n");
		return -EIO;
	}

	if (uart_irq_rx_ready(dev)) {
		uint8_t rc = getStructuredPacket(dev, &recv_packet, timeout);
				if (rc != FPS_OK) {
					return FPS_PACKETRECIEVEERROR;
				}
				if (recv_packet.type != FPS_ACKPACKET) {
					return FPS_PACKETRECIEVEERROR;
				}
	}
}

static int grow_r502a_init_interrupt(const struct device *dev)
{
	drv_data->work.handler = grow_r502a_work_cb;
	return 0;
}

static int grow_r502a_init(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;

	drv_data->uart_dev = device_get_binding(DT_INST_BUS_LABEL(0));

	if (!drv_data->uart_dev) {
		LOG_DBG("uart device is not found: %s",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	if (grow_r502a_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api grow_r502a_api = {
	.trigger_set = grow_r502a_trigger_set,
	.sample_fetch = grow_r502a_sample_fetch,
	.channel_get = grow_r502a_channel_get,
	.attr_set = grow_r502a_attr_set,
	.attr_get = grow_r502a_attr_get,
};

static struct grow_r502a_data grow_r502a_data;

DEVICE_DT_INST_DEFINE(0, &grow_r502a_init, NULL,
		    &grow_r502a_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &grow_r502a_api);
