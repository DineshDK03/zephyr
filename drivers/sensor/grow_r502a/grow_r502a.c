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

#include "grow_r502a.h"

LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

static const struct device *dev;

static void write_packet(uint8_t pkg_type, uint8_t *pkg_data, uint16_t len)
{
	struct grow_r502a_data *drv_data = dev->data;

	len += 2;

	uint8_t packet[] = {(uint8_t)(FPS_STARTCODE >> 8), (uint8_t)FPS_STARTCODE,
			    (uint8_t)(drv_data->address >> 24), (uint8_t)(drv_data->address >> 16),
			    (uint8_t)(drv_data->address >> 8), (uint8_t)(drv_data->address),
			    (uint8_t)pkg_type, (uint8_t)(len >> 8), (uint8_t)(len) };

	uart_fifo_fill(dev, packet, sizeof(packet));
	uint16_t sum = (len >> 8) + (len & 0xFF) + pkg_type;

	for (uint8_t i = 0; i < len - 2; i++) {
		uart_fifo_fill(dev, &pkg_data[i], 1);
		sum += pkg_data[i];
	}
	uart_fifo_fill(dev, (uint8_t *)(&sum) + 1, 1);
	uart_fifo_fill(dev, (uint8_t *)&sum, 1);
}

static int16_t get_reply(uint8_t *replyBuf, uint16_t buflen,
							uint8_t *pktid)
{
	struct grow_r502a_data *drv_data;
	uint16_t header = 0;
	uint8_t pid = 0;
	uint16_t length = 0;
	uint16_t chksum = 0;
	uint16_t remn = 0;
	uint16_t state = 0;

	int64_t timeout_time = k_uptime_get() + (int64_t)K_MSEC(DEFAULT_TIMEOUT);

	while (k_uptime_get() < timeout_time) {
		switch (state) {
		case 0: {
			uint8_t byte;

			uart_fifo_read(dev, &byte, 1);

			header <<= 8; header |= byte;
			if (header != FPS_STARTCODE) {
				break;
			}

			state++;
			header = 0;

			LOG_INF("\r\n[+]Got header");
			break;
		}
		case 1:
		case 2:
		case 3:
		case 4: {
			uart_fifo_read(dev, drv_data->buffer, 4);
			uint32_t addr = drv_data->buffer[0]; addr <<= 8;

			addr |= drv_data->buffer[1]; addr <<= 8;
			addr |= drv_data->buffer[2]; addr <<= 8;
			addr |= drv_data->buffer[3];
			if (addr != drv_data->address) {
				state = FPS_STATE_READ_HEADER;
				LOG_ERR("[+]Wrong address: 0x%X", addr);
				break;
			}
			state++;
			LOG_INF("[+]Address: 0x%X", addr);
			break;
		}
		case 5:
			uart_fifo_read(dev, &pid, 1);
			chksum = pid;
			*pktid = pid;
			state++;
			LOG_INF("[+]PID: 0x%X", pid);
			break;
		case 6:
		case 7: {
			uart_fifo_read(dev, drv_data->buffer, 2);
			length = drv_data->buffer[0]; length <<= 8;
			length |= drv_data->buffer[1];
			if (length > FPS_MAX_PACKET_LEN + 2 || (length > buflen + 2)) {
				state = FPS_STATE_READ_HEADER;
				LOG_ERR("[+]Packet too long: %d", length);
				continue;
			}
			/* num of bytes left to read */
			remn = length;
			chksum += drv_data->buffer[0]; chksum += drv_data->buffer[1];
			state++;
			LOG_INF("[+]Length: %d", length - 2);
			break;
		}
		case 8: {
			if (remn <= 2) {
				state = FPS_STATE_READ_CHECKSUM;
				break;
			}
			/* we now have to stop using 'drv_data->buffer' since
			 * we may be storing data in it now
			 */
			uint8_t byte;

			uart_fifo_read(dev, &byte, 1);
			*replyBuf++ = byte;
			chksum += byte;
			LOG_INF("%X");
			remn--;
			break;
		}
		case 9:
		case 10: {
			uint8_t temp[2];

			uart_fifo_read(dev, temp, 2);
			uint16_t to_check = temp[0]; to_check <<= 8;

			to_check |= temp[1];
			if (to_check != chksum) {
				state = FPS_STATE_READ_HEADER;
				LOG_ERR("\r\n[+]Wrong chksum: 0x%X", to_check);
				continue;
			}
			LOG_INF("\r\n[+]Read complete");
			/* without chksum */
			return length - 2;
		}
	}
	}

	LOG_ERR("[+]Response timeout\r\n");
	return FPS_TIMEOUT;
}

/* read standard ACK-reply into library grow_r502a_data->buffer and
 * return pkg_data length and confirmation code
 */
static int16_t read_ack_get_response(uint8_t *rc)
{
	struct grow_r502a_data *drv_data;
	uint8_t pkg_id = 0;
	int16_t len = get_reply(drv_data->buffer, FPS_BUFFER_SZ, &pkg_id);

	/* most likely timed out */
	if (len < 0)
		return len;

	/* wrong pkt id */
	if (pkg_id != FPS_ACKPACKET) {
		LOG_ERR("[+]Wrong PID: 0x%X", pkg_id);
		return FPS_READ_ERROR;
	}

	*rc = drv_data->buffer[0];

	/* minus confirmation code */
	return --len;
}

static int16_t check_sensor(void)
{
	struct grow_r502a_data *drv_data;

	drv_data->buffer[0] = FPS_CHECKSENSOR;
	write_packet(FPS_COMMANDPACKET, drv_data->buffer, 1);
	uint8_t confirm_code = 0;
	int16_t rc = read_ack_get_response(&confirm_code);

	if (rc < 0)
		return rc;

	return confirm_code;
}

static int16_t fps_get_image(void)
{
	struct grow_r502a_data *drv_data;

	drv_data->buffer[0] = FPS_GENIMAGE;
	write_packet(FPS_COMMANDPACKET, drv_data->buffer, 1);
	uint8_t confirm_code = 0;
	int16_t rc = read_ack_get_response(&confirm_code);

	if (rc < 0)
		return rc;

	return confirm_code;
}

static int16_t fps_image2Tz(uint8_t slot)
{
	struct grow_r502a_data *drv_data;

	drv_data->buffer[0] = FPS_IMAGE2TZ;
	drv_data->buffer[1] = slot;
	write_packet(FPS_COMMANDPACKET, drv_data->buffer, 2);
	uint8_t confirm_code = 0;
	int16_t rc = read_ack_get_response(&confirm_code);

	if (rc < 0) {
		return rc;
	}
	return confirm_code;
}

static int16_t fps_create_model(void)
{
	struct grow_r502a_data *drv_data;

	drv_data->buffer[0] = FPS_REGMODEL;
	write_packet(FPS_COMMANDPACKET, drv_data->buffer, 1);
	uint8_t confirm_code = 0;
	int16_t rc = read_ack_get_response(&confirm_code);

	if (rc < 0) {
		return rc;
	}

	return confirm_code;
}

static int16_t fps_store_model(uint16_t id, uint8_t slot)
{
	struct grow_r502a_data *drv_data;

	drv_data->buffer[0] = FPS_STORE;
	drv_data->buffer[1] = slot;
	drv_data->buffer[2] = id >> 8; drv_data->buffer[3] = id & 0xFF;

	write_packet(FPS_COMMANDPACKET, drv_data->buffer, 4);
	uint8_t confirm_code = 0;
	int16_t rc = read_ack_get_response(&confirm_code);

	if (rc < 0)
		return rc;

	return confirm_code;
}

static int grow_r502a_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{
	if (chan == SENSOR_CHAN_FINGERPRINT) {
		if (check_sensor() == FPS_OK) {
			LOG_INF("Sensor Check OK");
		} else {
			LOG_ERR("Sensor Check Error : 0x%X", check_sensor());
		}
	} else {
		return -ENOTSUP;
	}
}

static int grow_r502a_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct grow_r502a_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		val->val1 = drv_data->fingerID;
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
	const struct grow_r502a_data *drv_data = dev->data;

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
			return;
			}
			LOG_INF("Remove finger");
			K_MSEC(2000);
			p = 0;
			while (p != FPS_NOFINGER) {
				p = fps_get_image();
				K_MSEC(10);
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
				return;
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
	const struct grow_r502a_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_FINGERPRINT) {
		if (attr) { /*To Do*/
			/*To Do*/
		} else {
			return -ENOTSUP;
		}
	} else {
		return -ENOTSUP;
	}
}

static int grow_r502a_trigger_set(const struct device *dev,
				 const struct sensor_trigger *trig,
				 sensor_trigger_handler_t handler)
{

}

/* int grow_r502a_init_interrupt(const struct device *dev) */
/* { */
/*  */
/* } */

/* static void uart_isr(const struct device *dev, void *user_data) */
/* { */
/* ARG_UNUSED(user_data); */
/*  */
/* while (uart_irq_update(dev) && uart_irq_is_pending(dev)) { */
/* if (uart_irq_tx_ready(dev)) { */
/* tx_callback(); */
/* } */
/* } */
/* } */

static const struct sensor_driver_api grow_r502a_api = {
	/* .trigger_set = grow_r502a_trigger_set, */
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

	drv_data->fingerID = 0;
	/* uart_irq_callback_set(dev, uart_isr); */
	return 0;
}

static struct grow_r502a_data grow_r502a_data;

DEVICE_DT_INST_DEFINE(0, &grow_r502a_init, NULL,
		    &grow_r502a_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &grow_r502a_api);
