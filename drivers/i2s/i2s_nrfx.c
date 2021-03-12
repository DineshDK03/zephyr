/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <drivers/i2s.h>
#include <logging/log.h>
#include <nrfx_i2s.h>
#include <stdlib.h>

#include "i2s_nrfx.h"

LOG_MODULE_REGISTER(i2s_nrfx, CONFIG_I2S_LOG_LEVEL);

#define DEV_CFG(dev) \
	(const struct i2s_nrfx_config *const)((dev)->config)
#define DEV_DATA(dev) \
	((struct i2s_nrfx_data *const)(dev)->data)

#define LOG_ERROR(msg, ch_state) LOG_ERR("[Ch state: %u]%s", ch_state, msg)

/* Interface service functions */
static inline struct i2s_nrfx_data *get_interface(void);
static int interface_set_state(struct i2s_nrfx_data *i2s,
			       enum i2s_if_state new_state);

static void interface_error_service(struct i2s_nrfx_data *i2s,
				    const char *const err_msg)
{
	assert(i2s != NULL);
	interface_set_state(i2s, I2S_IF_ERROR);
	LOG_ERR("%s", err_msg);
	nrfx_i2s_stop();
}

static int interface_set_state(struct i2s_nrfx_data *i2s,
			       enum i2s_if_state new_state)
{
	bool change_forbidden = false;

	assert(i2s != NULL);
	switch (new_state) {
	case I2S_IF_STOPPING:
		if (i2s->state != I2S_IF_RUNNING &&
		    i2s->state != I2S_IF_NEEDS_RESTART) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_NEEDS_RESTART:
		if (i2s->state != I2S_IF_RUNNING)
			change_forbidden = true;
		break;
	case I2S_IF_RUNNING:
		if (i2s->state != I2S_IF_RESTARTING &&
		    i2s->state != I2S_IF_READY) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_READY:
		if (i2s->state != I2S_IF_STOPPING &&
		    i2s->state != I2S_IF_NOT_READY &&
		    i2s->state != I2S_IF_ERROR) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_RESTARTING:
		if (i2s->state != I2S_IF_NEEDS_RESTART)
			change_forbidden = true;
		break;
	case I2S_IF_NOT_READY:
		nrfx_i2s_uninit();
		break;
	case I2S_IF_ERROR:
		break;
	default:
		LOG_ERR("Invalid interface state chosen");
		return -EINVAL;
	}
	if (change_forbidden) {
		interface_error_service(i2s,
					"Failed to change interface state");
		return -EIO;
	}
	i2s->state = new_state;
	return 0;
}

static inline enum i2s_if_state interface_get_state(
	struct i2s_nrfx_data *i2s)
{
	assert(i2s != NULL);
	return i2s->state;
}

static int interface_start(struct i2s_nrfx_data *i2s)
{
	int ret;

	ret = interface_set_state(i2s, I2S_IF_RUNNING);
	if (ret < 0)
		return ret;

	/*nrfx_i2s_start() procedure needs buffer size in 32-bit word units*/
	nrfx_err_t status = nrfx_i2s_start(&i2s->buffers,
					   i2s->size / sizeof(uint32_t), 0);

	if (status != NRFX_SUCCESS) {
		interface_error_service(i2s, "Failed to start interface");
		ret = -EIO;
	}

	return ret;
}

static int interface_stop(struct i2s_nrfx_data *i2s)
{
	int ret;

	ret = interface_set_state(i2s, I2S_IF_STOPPING);
	if (ret < 0) {
		interface_error_service(i2s, "Failed to stop interface");
		return ret;
	}
	return 0;
}

static int interface_restart(struct i2s_nrfx_data *i2s)
{
	return interface_set_state(i2s, I2S_IF_NEEDS_RESTART);
}

static int interface_stop_restart(struct i2s_nrfx_data *i2s,
				  struct stream *stream_to_stop_restart,
				  enum i2s_state other_stream_state)
{
	int ret;

	stream_to_stop_restart->state = I2S_STATE_STOPPING;
	if (stream_to_stop_restart->state != I2S_STATE_STOPPING)
		return stream_to_stop_restart->state;
	if (other_stream_state == I2S_STATE_RUNNING) {
		ret = interface_restart(i2s);
		if (ret < 0)
			return ret;
	} else {
		ret = interface_stop(i2s);
		if (ret < 0) {
			interface_error_service(i2s,
						"Failed to restart interface");
			return ret;
		}
	}
	return 0;
}

static inline bool next_buffers_needed(uint32_t status)
{
	return (status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
	       == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED;
}

static inline bool transfer_stopped(uint32_t status)
{
	return (status & NRFX_I2S_STATUS_TRANSFER_STOPPED)
	       == NRFX_I2S_STATUS_TRANSFER_STOPPED;
}

static void stream_tx_callback(struct i2s_nrfx_data *i2s,
			       nrfx_i2s_buffers_t const *p_released,
			       uint32_t status, nrfx_i2s_buffers_t *p_new_buffers);
static void stream_rx_callback(struct i2s_nrfx_data *i2s,
			       nrfx_i2s_buffers_t const *p_released,
			       uint32_t status, nrfx_i2s_buffers_t *p_new_buffers);

static struct k_work_q i2s_work_queue;
static K_KERNEL_STACK_DEFINE(i2s_work_queue_stack,
			     CONFIG_NRFX_I2S_WORK_QUEUE_STACK_SIZE);

static void i2s_work_handler(struct k_work *work)
{
	struct i2s_nrfx_data *i2s = get_interface();
	struct stream *rx_str = &i2s->rx;
	struct stream *tx_str = &i2s->tx;
	nrfx_i2s_buffers_t p_new_buffers;

	/* Call callbacks for tx/rx streams if they are not in idle state*/
	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;
	if (rx_str->state != I2S_STATE_READY &&
			rx_str->state != I2S_STATE_NOT_READY) {
		stream_rx_callback(i2s, i2s->p_released, i2s->status,
					 &p_new_buffers);
	}
	if (tx_str->state != I2S_STATE_READY &&
			tx_str->state != I2S_STATE_NOT_READY) {
		stream_tx_callback(i2s, i2s->p_released, i2s->status,
					 &p_new_buffers);
	}
	if (next_buffers_needed(i2s->status)) {
		if (interface_get_state(i2s) == I2S_IF_NEEDS_RESTART ||
				interface_get_state(i2s) == I2S_IF_STOPPING) {
			/* if driver needs new buffers but user requested
			 * interface state change (e.g. called i2s_trigger())
			 * then peripheral needs to be stopped. In this case
			 * there is no need to set new buffers for driver.
			 * On next callback execution (this one will be caused
			 * by 'EVENT_STOPPED' - look 'else' below) interface
			 * will change state to:
			 *  - 'I2S_IF_RESTARTING' if there is at least one
			 *    stream involved in transmission
			 *  - 'I2S_IF_STOPPING' if no more data transmission
			 *    needed
			 */
			nrfx_i2s_stop();
			return;
		} else if (interface_get_state(i2s) == I2S_IF_RUNNING) {
			/* if driver requested new buffers and interface works
			 * normally then just set them
			 * (store 'TXD.PTR'/'RXD.PTR' registers)
			 */
			if (nrfx_i2s_next_buffers_set(&p_new_buffers) !=
					NRFX_SUCCESS) {
				interface_error_service(i2s,
							"Internal service error");
				return;
			}
		}
		i2s->buffers = p_new_buffers;
	} else {
		if (interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
			if (interface_set_state(i2s, I2S_IF_RESTARTING) != 0) {
				interface_error_service(i2s,
							"Internal service error");
			}
		} else if (interface_get_state(i2s) == I2S_IF_STOPPING) {
			if (interface_set_state(i2s, I2S_IF_READY)) {
				interface_error_service(i2s,
							"Internal service error");
			}
		} else if (rx_str->state != I2S_STATE_RUNNING &&
				 tx_str->state != I2S_STATE_RUNNING) {
			if (interface_get_state(i2s) == I2S_IF_RUNNING)
				interface_stop(i2s);
		}
	}
	/* if nrfx driver sets 'NRFX_I2S_STATUS_TRANSFER_STOPPED' flag and the
	 * interface state is 'I2S_IF_RESTARTING' it means that last transfer
	 * before restart occurred. The peripheral will be stopped and started
	 * again (the reason could be e.g. start rx while tx works)
	 */
	if (transfer_stopped(i2s->status) &&
			interface_get_state(i2s) == I2S_IF_RESTARTING) {
		int ret = interface_start(i2s);

		if (ret < 0)
			interface_error_service(i2s, "Internal ISR error");

		return;
	}
}

/* this function is called by 'nrfx_i2s_irq_handler' which delivers:
 *  - 'p_released' -	set of rx/tx buffers with received/sent data
 *  - 'status		 -	bit field, at the moment:
 *      if NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED (1) is set: driver needs new
 *              buffers ('EVENT_TXPTRUPD' or 'EVENT_RXPTRUPD' is active)
 *      if NRFX_I2S_STATUS_TRANSFER_STOPPED (2) is set: driver has finished
 *              transmission ('EVENT_STOPPED' is active)
 */
static void interface_handler(nrfx_i2s_buffers_t const *p_released,
			      uint32_t status)
{
	struct i2s_nrfx_data *i2s = get_interface();

	i2s->status = status;
	i2s->p_released = p_released;
	k_work_submit_to_queue(&i2s_work_queue, &i2s->i2s_work);
}

/* stream management functions */
static struct stream *dir_stream_get(struct i2s_nrfx_data *i2s,
				     enum i2s_dir dir)
{
	assert(i2s != NULL);
	switch (dir) {
	case I2S_DIR_RX:
		return &i2s->rx;
	case I2S_DIR_TX:
		return &i2s->tx;
	}
	return NULL;
}

static int stream_tx_get_data(struct i2s_nrfx_data *i2s, uint32_t **buf)
{
	int ret;
	struct stream *ch_tx = &i2s->tx;
	void *buffer;

	assert(i2s != NULL);
	ret = k_msgq_get(&ch_tx->queue, &buffer, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Queue Get Error\n");
		return ret;
	}
	*buf = buffer;
	k_sem_give(&ch_tx->sem);
	return 0;
}

static int stream_rx_store_data(struct i2s_nrfx_data *i2s, uint32_t **buf)
{
	int ret;
	void *buffer;
	struct stream *ch_rx = &i2s->rx;

	assert(i2s != NULL);
	ret = k_msgq_put(&ch_rx->queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to put buffer in Output queue\n");
		return ret;
	}
	*buf = buffer;
	k_sem_give(&ch_rx->sem);

	return 0;
}

static void stream_tx_mem_clear(struct i2s_nrfx_data *i2s,
				void const *first_block)
{
	struct stream *const ch_tx = &i2s->tx;
	void *mem_block = (void *)first_block;

	assert(i2s != NULL);
	if (first_block == NULL) {
		if (stream_tx_get_data(i2s, (uint32_t **)&mem_block) != 0)
			return;
	}

	do {
		k_mem_slab_free(ch_tx->mem_slab,
				(void **)&mem_block);
	} while (stream_tx_get_data(i2s, (uint32_t **)&mem_block) == 0);

	while (ch_tx->sem.count < ch_tx->sem.limit)
		k_sem_give(&ch_tx->sem);
}

static void stream_rx_mem_clear(struct i2s_nrfx_data *i2s)
{
	void *mem_block;
	struct stream *const ch_rx = &i2s->rx;

	assert(i2s != NULL);
	while (k_msgq_get(&ch_rx->queue,
			   (void **)&mem_block, K_NO_WAIT) == 0) {
		k_mem_slab_free(ch_rx->mem_slab, (void **)&mem_block);
	}
	while (ch_rx->sem.count) {
		if (k_sem_take(&ch_rx->sem, K_NO_WAIT) < 0)
			return;
	}
}

static void stream_mem_clear(struct i2s_nrfx_data *i2s, enum i2s_dir dir)
{
	if (dir == I2S_DIR_RX)
		stream_rx_mem_clear(i2s);
	else
		stream_tx_mem_clear(i2s, NULL);
}

static int stream_start(struct i2s_nrfx_data *i2s, enum i2s_dir dir)
{
	int ret;
	unsigned int key;
	struct stream *const stream_to_start = dir_stream_get(i2s, dir);

	if (interface_get_state(i2s) != I2S_IF_RUNNING &&
	    interface_get_state(i2s) != I2S_IF_READY) {
		LOG_ERR("Invalid interface state");
		return -EIO;
	}

	if (dir == I2S_DIR_RX) {
		ret = k_mem_slab_alloc(stream_to_start->mem_slab,
				       (void **)&i2s->buffers.p_rx_buffer, K_NO_WAIT);
	} else {
		ret = stream_tx_get_data(i2s,
					 (uint32_t **)&i2s->buffers.p_tx_buffer);
	}
	if (ret < 0) {
		LOG_ERROR(dir == I2S_DIR_RX ? "Memory allocation error" :
			  "Queue fetching error",
			  (uint32_t)stream_to_start->state);
		return ret;
	}

	key = irq_lock();
	if (interface_get_state(i2s) == I2S_IF_RUNNING)
		ret = interface_restart(i2s);
		else if (interface_get_state(i2s) == I2S_IF_READY)
			ret = interface_start(i2s);

	irq_unlock(key);

	if (ret < 0) {
		LOG_ERROR("Failed to start/restart interface",
			  (uint32_t)stream_to_start->state);
		k_mem_slab_free(stream_to_start->mem_slab,
				       (void **)&i2s->buffers.p_rx_buffer);
		return ret;
	}

	return 0;
}

static void cfg_reinit(struct i2s_nrfx_data *i2s);

static int stream_drop(struct i2s_nrfx_data *i2s, enum i2s_dir dir)
{
	int ret;
	struct stream *const stream_to_drop = dir_stream_get(i2s, dir);
	struct stream *const other_stream = dir_stream_get(i2s,
						       dir == I2S_DIR_TX ? I2S_DIR_RX : I2S_DIR_TX);

	if (stream_to_drop->state == I2S_STATE_RUNNING) {
		ret =  interface_stop_restart(i2s, stream_to_drop,
					      other_stream->state);
		if (ret < 0) {
			interface_error_service(i2s,
						"Failed to restart interface");
			return ret;
		}
	} else {
		cfg_reinit(i2s);
		stream_mem_clear(i2s, dir);
	}
	return 0;
}

static int stream_stop(struct i2s_nrfx_data *i2s, enum i2s_dir dir)
{
	struct stream *const stream_to_stop = dir_stream_get(i2s, dir);
	struct stream *const other_stream = dir_stream_get(i2s,
						       dir == I2S_DIR_TX ? I2S_DIR_RX : I2S_DIR_TX);

	return interface_stop_restart(i2s, stream_to_stop,
				      other_stream->state);
}

static int stream_drain(struct i2s_nrfx_data *i2s, enum i2s_dir dir)
{
	int ret;
	struct stream *const stream_to_drain = dir_stream_get(i2s, dir);

	if (dir == I2S_DIR_RX) {
		ret = stream_stop(i2s, dir);
		return ret;
	}
	if (dir == I2S_DIR_TX)
		stream_to_drain->state = I2S_STATE_STOPPING;

	return 0;
}

/* @brief	Callback for tx
 *			In case of constant tx transmission this callback:
 *			- frees tx buffer which has just been transmitted via I2S interface
 *			- gets new buffer from queue which will be passed to nrfx driver as
 *		next to be transferred (content of 'TXD.PTR' register).
 *   @param[in] Status  informs about handler execution reason:
 *					- if 1 - next buffer is needed
 *					- if 0 - transfer is finishing
 */
static void stream_tx_callback(struct i2s_nrfx_data *i2s,
			       nrfx_i2s_buffers_t const *p_released,
			       uint32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct stream *ch_tx = &i2s->tx;
	uint32_t *mem_block = NULL;
	int get_data_ret = 1;

	if (ch_tx->state == I2S_STATE_RUNNING &&
	    interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
		/* This code services case when tx stream transmits data
		 * constantly while rx stream is beginning/finishing its
		 * transfer (user called' i2s_trigger()' with 'I2S_DIR_RX'
		 * parameter)).
		 * In this case NRF I2S peripheral needs to be restarted.
		 */
		if (p_released->p_tx_buffer != NULL &&
		    next_buffers_needed(status)) {
			/* when interface needs to be restarted and
			 * last event was 'EVENT_STOPPED' then we don't
			 * free this buffer - it will be used after
			 * interface restarts - tx transmission
			 * will be still running and user don't want to
			 * lose data.
			 */
			k_mem_slab_free(ch_tx->mem_slab,
					(void **)&p_released->p_tx_buffer);
		}
		get_data_ret = stream_tx_get_data(i2s, &mem_block);
		if (get_data_ret == 0) {
			k_mem_slab_free(ch_tx->mem_slab,
					(void **)&mem_block);
		}
		return;
	}

	if (p_released->p_tx_buffer != NULL) {
		k_mem_slab_free(ch_tx->mem_slab,
				(void **)&p_released->p_tx_buffer);
	}

	if (next_buffers_needed(status))
		get_data_ret = stream_tx_get_data(i2s, &mem_block);

	if (ch_tx->state == I2S_STATE_STOPPING) {
		/* finishing tx transfer caused by user trigger command*/
		enum i2s_trigger_cmd ch_cmd = ch_tx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				if (get_data_ret == 0)
					stream_tx_mem_clear(i2s, mem_block);
				break;
			case I2S_TRIGGER_STOP:
				if (get_data_ret == 0)
					k_mem_slab_free(ch_tx->mem_slab,
							(void **)&mem_block);
				break;
			case I2S_TRIGGER_DRAIN:
				break;
			default:
				ch_tx->state = I2S_STATE_ERROR;
				return;
			}
		} else {
			ch_tx->state = I2S_STATE_READY;

			if (ch_tx->state != I2S_STATE_READY) {
				ch_tx->state = I2S_STATE_ERROR;
				return;
			}
			if (ch_cmd == I2S_TRIGGER_DRAIN)
				return;
		}
		i2s->buffers.p_tx_buffer = NULL;
	} else if (ch_tx->state == I2S_STATE_ERROR) {
		return;
	} else if (get_data_ret < 0) {
		interface_error_service(i2s, "TX internal callback error");
		ch_tx->state = I2S_STATE_ERROR;
		return;
	}

	/* continue transmission */
	p_new_buffers->p_tx_buffer = mem_block;
}

/* @brief	Callback for rx.
 *			In case of constant rx transmission this callback:
 *			- stores in queue rx buffer which has just been received
 *			- via I2S interface
 *			- allocates new rx buffer for next transfer purpose
 * @param[in] Status   informs about handler execution reason:
 *						- if 1 - next buffer is needed
 *					- if 0 - transfer is finishing
 */
static void stream_rx_callback(struct i2s_nrfx_data *i2s,
			       nrfx_i2s_buffers_t const *p_released,
			       uint32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct stream *ch_rx = &i2s->rx;
	int ret;

	if (p_released->p_rx_buffer != NULL && next_buffers_needed(status)) {
		/* Content of received buffer is valuable.
		 * If 'EVENT_STOPPED' generated 'then next_buffers_needed()'
		 * returns false - function 'stream_rx_store_data()' won't
		 * execute because buffer didn't fill.
		 */
		ret = stream_rx_store_data(i2s,
					   (uint32_t **)&p_released->p_rx_buffer);
		if (ret < 0)
			return;
	}

	if (ch_rx->state == I2S_STATE_STOPPING) {
		/* finishing rx transfer caused by user trigger command*/
		enum i2s_trigger_cmd ch_cmd = ch_rx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				stream_rx_mem_clear(i2s);
				break;
			case I2S_TRIGGER_DRAIN:
			case I2S_TRIGGER_STOP:
				break;
			default:
				LOG_ERROR("RX Callback command unknown",
					  (uint32_t)ch_rx->state);
				ch_rx->state = I2S_STATE_ERROR;
				return;
			}
		} else {
			if (p_released->p_rx_buffer) {
				ret = stream_rx_store_data(i2s,
							   (uint32_t **)&p_released->p_rx_buffer);
				if (ret < 0)
					return;
			}
			ch_rx->state = I2S_STATE_READY;
			if (ch_rx->state != I2S_STATE_READY) {
				ch_rx->state = I2S_STATE_ERROR;
				return;
			}
		}
		i2s->buffers.p_rx_buffer = NULL;
		return;
	} else if (ch_rx->state == I2S_STATE_RUNNING &&
		   interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
		return;
	} else if (ch_rx->state == I2S_STATE_ERROR) {
		if (p_released->p_rx_buffer != NULL) {
			k_mem_slab_free(ch_rx->mem_slab,
					(void **)&p_released->p_rx_buffer);
		}
		return;
	}

	if (next_buffers_needed(status)) {
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void **)&p_new_buffers->p_rx_buffer,
				       K_NO_WAIT);
		if (ret < 0) {
			/*overrun error occurred*/
			interface_error_service(i2s, "RX overrun error");
			ch_rx->state = I2S_STATE_ERROR;
			return;
		}
	}
}

/* configuration functions */
static void cfg_reinit(struct i2s_nrfx_data *i2s)
{
	struct stream *ch_tx = &i2s->tx;
	struct stream *ch_rx = &i2s->rx;

	nrfx_i2s_stop();
	i2s->state = I2S_IF_READY;
	ch_tx->state = I2S_STATE_READY;
	ch_rx->state = I2S_STATE_READY;
}

static inline nrf_i2s_mck_t cfg_get_divider(
	struct i2s_clk_settings_t const *clk_set, uint8_t word_size)
{
	uint32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->divider[sub_idx];
}

static inline nrf_i2s_ratio_t cfg_get_ratio(
	struct i2s_clk_settings_t const *clk_set, uint8_t word_size)
{
	uint32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->ratio[sub_idx];
}

static void cfg_match_clock_settings(nrfx_i2s_config_t *config,
				     struct i2s_config const *i2s_cfg)
{
	const struct i2s_clk_settings_t i2s_clock_settings[] =
		NRFX_I2S_AVAILABLE_CLOCK_SETTINGS;
	uint32_t des_s_r = (int32_t)i2s_cfg->frame_clk_freq;
	uint8_t nb_of_settings_array_elems = ARRAY_SIZE(i2s_clock_settings);
	struct i2s_clk_settings_t const *chosen_settings =
		&i2s_clock_settings[nb_of_settings_array_elems - 1];

	for (uint8_t i = 1; i < nb_of_settings_array_elems; ++i) {
		if (des_s_r < i2s_clock_settings[i].frequency) {
			uint32_t diff_h =
				i2s_clock_settings[i].frequency - des_s_r;
			uint32_t diff_l =
				abs((int32_t)des_s_r -
				    (int32_t)i2s_clock_settings[i - 1].frequency);
			chosen_settings = (diff_h < diff_l) ?
					  (&i2s_clock_settings[i]) :
					  (&i2s_clock_settings[i - 1]);
			break;
		}
	}
	config->mck_setup = cfg_get_divider(chosen_settings,
					    i2s_cfg->word_size);
	config->ratio = cfg_get_ratio(chosen_settings,
				      i2s_cfg->word_size);
}

static int cfg_periph_config(const struct device *dev,
			     nrfx_i2s_config_t *const drv_cfg,
			     struct i2s_config const *i2s_cfg)
{
	struct i2s_nrfx_data *i2s = get_interface();
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);

	assert(drv_cfg != NULL);
	drv_cfg->sck_pin = dev_const_cfg->sck_pin;
	drv_cfg->lrck_pin = dev_const_cfg->lrck_pin;
	drv_cfg->mck_pin = dev_const_cfg->mck_pin;
	drv_cfg->sdout_pin = dev_const_cfg->sdout_pin;
	drv_cfg->sdin_pin = dev_const_cfg->sdin_pin;
	if (i2s_cfg->mem_slab == NULL) {
		interface_error_service(i2s, "Config: Invalid memory slab");
		return -EINVAL;
	}

	/*configuration validity verification*/
	switch (i2s_cfg->word_size) {
	case 8U:
		drv_cfg->sample_width = NRF_I2S_SWIDTH_8BIT;
		break;
	case 16U:
		drv_cfg->sample_width = NRF_I2S_SWIDTH_16BIT;
		break;
	case 24U:
		drv_cfg->sample_width = NRF_I2S_SWIDTH_24BIT;
		break;
	default:
		if (i2s_cfg->word_size < 8 || i2s_cfg->word_size > 32) {
			/*this value isn't compatible with I2S standard*/
			interface_error_service(i2s,
						"Config: Invalid word size");
			return -EINVAL;
		}
		interface_error_service(i2s,
					"Config: Unsupported word size");
		return -ENOTSUP;
	}

	/*format validity verification*/
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		drv_cfg->alignment = NRF_I2S_ALIGN_LEFT;
		drv_cfg->format = NRF_I2S_FORMAT_I2S;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		drv_cfg->alignment = NRF_I2S_ALIGN_LEFT;
		drv_cfg->format = NRF_I2S_FORMAT_ALIGNED;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		drv_cfg->alignment = NRF_I2S_ALIGN_RIGHT;
		drv_cfg->format = NRF_I2S_FORMAT_ALIGNED;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		interface_error_service(i2s,
					"Config: Unsupported data format");
		return -ENOTSUP;
	default:
		interface_error_service(i2s, "Config: Invalid data format");
		return -EINVAL;
	}
	if (i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) {
		interface_error_service(i2s,
					"Config: Unsupported clock format");
		return -ENOTSUP;
	}

	/*mode options validity check*/
	if ((i2s_cfg->options & I2S_OPT_PINGPONG) ||
	    (i2s_cfg->options & I2S_OPT_LOOPBACK)) {
		interface_error_service(i2s,
					"Config: Unsupported mode setiings");
		return -ENOTSUP;
	}

	if (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
		if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) &&
		    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
			drv_cfg->mode = NRF_I2S_MODE_SLAVE;
		} else {
			if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
			    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
				interface_error_service(i2s,
							"Config: Unsupported mode setiings");
				return -ENOTSUP;
			}
			drv_cfg->mode = NRF_I2S_MODE_MASTER;
		}
	} else {
		interface_error_service(i2s,
					"Config: Unsupported clock settings");
		return -ENOTSUP;
	}

	/*channel and size configuration validity check*/
	switch (i2s_cfg->channels) {
	case 2:
		drv_cfg->channels = NRF_I2S_CHANNELS_STEREO;
		break;
	case 1:
		drv_cfg->channels = NRF_I2S_CHANNELS_LEFT;
		break;
	default:
		interface_error_service(i2s,
					"Config: Invalid number of channels");
		return -EINVAL;
	}
	if (i2s_cfg->block_size == 0) {
		interface_error_service(i2s, "Config: Invalid block size");
		return -EINVAL;
	}
	i2s->size = i2s_cfg->block_size;
	cfg_match_clock_settings(drv_cfg, i2s_cfg);

	return 0;
}

/* Zephyr I2S Driver API functions */
static int i2s_nrfx_initialize(const struct device *dev)
{
	struct i2s_nrfx_data *i2s = get_interface();
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);

	assert(dev != NULL);
	k_sem_init(&i2s->rx.sem, 0, CONFIG_NRFX_I2S_RX_BLOCK_COUNT);
	k_sem_init(&i2s->tx.sem, CONFIG_NRFX_I2S_TX_BLOCK_COUNT,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT);
	dev_const_cfg->irq_config(dev);
	k_work_q_start(&i2s_work_queue, i2s_work_queue_stack,
		K_KERNEL_STACK_SIZEOF(i2s_work_queue_stack),
		CONFIG_SYSTEM_WORKQUEUE_PRIORITY);
	k_work_init(&i2s->i2s_work, i2s_work_handler);
	/* Initialize the buffer queues */
	k_msgq_init(&i2s->tx.queue, (char *)i2s->tx.tx_msgs,
			sizeof(void *), CONFIG_NRFX_I2S_TX_BLOCK_COUNT);
	k_msgq_init(&i2s->rx.queue, (char *)i2s->rx.rx_msgs,
			sizeof(void *), CONFIG_NRFX_I2S_RX_BLOCK_COUNT);
	return 0;
}

static int i2s_nrfx_configure(const struct device *dev, enum i2s_dir dir,
			      struct i2s_config *i2s_cfg)
{
	int ret;
	nrfx_err_t status;
	struct stream *stream;
	nrfx_i2s_config_t drv_cfg;
	struct i2s_nrfx_data *i2s = get_interface();
	const struct stream *other_stream = dir_stream_get(i2s,
						      dir == I2S_DIR_TX ? I2S_DIR_RX : I2S_DIR_TX);

	assert(i2s_cfg != NULL && dev != NULL);
	stream = dir_stream_get(i2s, dir);
	assert(stream != NULL);

	/*for proper configuration transmission must be stopped */
	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERROR("Config: stream must be in ready/not ready state",
			  (uint32_t)stream->state);
		interface_error_service(i2s,
					"Config: Invalid stream state");
		return -EIO;
	}

	if (interface_get_state(i2s) != I2S_IF_READY &&
	    interface_get_state(i2s) != I2S_IF_NOT_READY) {
		LOG_ERROR("Config: Interface must be ready/not ready state",
			  (uint32_t)stream->state);
		return -EIO;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		/*reinit mode - cleaning stream data*/
		stream_mem_clear(i2s, dir);
		stream->state = I2S_STATE_NOT_READY;
		interface_set_state(i2s, I2S_IF_NOT_READY);
		return 0;
	}

	if (other_stream->state != I2S_STATE_NOT_READY) {
		const struct i2s_config *other_ch_cfg = &other_stream->config;

		/*if another stream is already configured it is necessary to
		 * check configuration compatibility
		 */
		if (other_ch_cfg->word_size      != i2s_cfg->word_size      ||
		    other_ch_cfg->channels       != i2s_cfg->channels       ||
		    other_ch_cfg->format         != i2s_cfg->format         ||
		    other_ch_cfg->options        != i2s_cfg->options        ||
		    other_ch_cfg->frame_clk_freq != i2s_cfg->frame_clk_freq ||
		    other_ch_cfg->block_size     != i2s_cfg->block_size) {
			LOG_ERR("Config: Incompatible stream settings");
			return -EINVAL;
		}
	} else {
		/* Single stream reinitialisation
		 * When reintialisation with two channels is needed it is
		 * necessary to deinit at least one of them (call this function
		 * with 'frame_clk_freq' set to 0)
		 */
		stream->state = I2S_STATE_NOT_READY;
		ret = interface_set_state(i2s, I2S_IF_NOT_READY);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			return -EIO;
		}
	}

	if (interface_get_state(i2s) == I2S_IF_NOT_READY) {
		/* peripheral configuration and driver initialization is needed
		 * only when interface is not configured (I2S_IF_NOT_READY)
		 */
		ret = cfg_periph_config(dev, &drv_cfg, i2s_cfg);
		/* disable streams in case of invalid configuration*/
		if (ret < 0) {
			LOG_ERROR("Config: Failed to configure peripheral",
				  (uint32_t)stream->state);
			stream->state = I2S_STATE_ERROR;
			return ret;
		}

		status = nrfx_i2s_init(&drv_cfg, interface_handler);
		if (status != NRFX_SUCCESS) {
			LOG_ERROR("Config: nrfx_i2s_init() returned value:",
				  (uint32_t)stream->state);
			LOG_ERR("0x%x", status);
			stream->state = I2S_STATE_ERROR;
			return -EIO;
		}
		ret = interface_set_state(i2s, I2S_IF_READY);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			return -EIO;
		}
	}
	stream->state = I2S_STATE_READY;

	/*store configuration*/
	if (dir == I2S_DIR_RX)
		i2s->buffers.p_rx_buffer = NULL;
	if (dir == I2S_DIR_TX)
		i2s->buffers.p_tx_buffer = NULL;
	stream->mem_slab = i2s_cfg->mem_slab;
	stream->timeout = i2s_cfg->timeout;
	stream->config = *i2s_cfg;
	return 0;
}

static struct i2s_config *i2s_nrfx_config_get(const struct device *dev,
					      enum i2s_dir dir)
{
	struct i2s_nrfx_data *i2s = get_interface();

	assert(dev != NULL);
	return &dir_stream_get(i2s, dir)->config;
}

static int i2s_nrfx_trigger(const struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	int ret;
	struct i2s_nrfx_data *i2s = get_interface();
	struct stream *stream;


	assert(dev != NULL);
	if ((interface_get_state(i2s) == I2S_IF_STOPPING) ||
	    (interface_get_state(i2s) == I2S_IF_NEEDS_RESTART)) {
		if (cmd != I2S_TRIGGER_PREPARE) {
			/* This case is not an error - it only provides
			 * that user can't trigger at the moment due to the
			 * unstable interface state (it's just changing).
			 * User should call it again after while. The API
			 * doesn't provide return value for this case.
			 */
			LOG_INF("Wait for stable state");
			return -EAGAIN;
		}
	}

	stream = dir_stream_get(i2s, dir);

	assert(stream != NULL);

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_START",
				  (uint32_t)stream->state);
			return -EIO;
		}
		ret = stream_start(i2s, dir);
		stream->state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_STOP",
				  (uint32_t)stream->state);
			return -EIO;
		}
		ret = stream_stop(i2s, dir);
		break;
	case I2S_TRIGGER_DRAIN:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_DRAIN",
				  (uint32_t)stream->state);
			return -EIO;
		}
		ret = stream_drain(i2s, dir);
		break;
	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_DROP",
				  (uint32_t)stream->state);
			return -EIO;
		}
		ret = stream_drop(i2s, dir);
		break;
	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_PREPARE",
				  (uint32_t)stream->state);
			return -EIO;
		}
		ret = stream_drop(i2s, dir);
		break;
	default:
		LOG_ERROR("Invalid trigger command",
			  (uint32_t)stream->state);
		return -EINVAL;
	}

	if (ret < 0) {
		LOG_ERROR("Error trigger while execution",
			  (uint32_t)stream->state);
		stream->state = I2S_STATE_ERROR;
		return ret;
	}
	stream->last_trigger_cmd = cmd;
	return 0;
}

static int i2s_nrfx_read(const struct device *dev, void **mem_block,
	 size_t *size)
{
	int ret = 0;
	struct i2s_nrfx_data *i2s = get_interface();
	struct stream *ch_rx = &i2s->rx;
	void *buffer;

	assert(dev != NULL && mem_block != NULL && size != NULL);
	if ((ch_rx->state == I2S_STATE_NOT_READY ||
	     ch_rx->state == I2S_STATE_ERROR)) {
		return -EIO;
	}

	ret = k_sem_take(&ch_rx->sem, SYS_TIMEOUT_MS(ch_rx->timeout));
	if (ret < 0)
		return ret;

	ret = k_msgq_get(&ch_rx->queue, &buffer, SYS_TIMEOUT_MS(ch_rx->timeout));
	if (ret != 0)
		return -EAGAIN;

	*mem_block = buffer;
	*size = i2s->size;
	return 0;
}

static int i2s_nrfx_write(const struct device *dev, void *mem_block,
	 size_t size)
{
	int ret;
	struct i2s_nrfx_data *i2s = get_interface();
	struct stream *ch_tx = &i2s->tx;

	assert(dev != NULL && mem_block != NULL);
	if (ch_tx->state != I2S_STATE_READY &&
	    ch_tx->state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	if (size != i2s->size) {
		LOG_ERR("Invalid size");
		return -EINVAL;
	}

	ret = k_sem_take(&ch_tx->sem, SYS_TIMEOUT_MS(ch_tx->timeout));
	if (ret < 0)
		return ret;

	ret = k_msgq_put(&ch_tx->queue, &mem_block, SYS_TIMEOUT_MS(ch_tx->timeout));
	if (ret) {
		LOG_ERR("k_msgq_put failed %d", ret);
		return ret;
	}
	return ret;
}

static const struct i2s_driver_api i2s_nrf_driver_api = {
	.configure = i2s_nrfx_configure,
	.config_get = i2s_nrfx_config_get,
	.read = i2s_nrfx_read,
	.trigger = i2s_nrfx_trigger,
	.write = i2s_nrfx_write,
};

#define I2S_NRFX_DEVICE(idx)								    \
static void i2s_nrfx_irq_##idx##_config(const struct device *dev);		    \
												\
	static inline struct i2s_nrfx_data *get_interface(void)	\
	{											\
		return DEV_DATA(DEVICE_DT_GET(DT_NODELABEL(i2s##idx)));			    \
	}										    \
											    \
	static const struct i2s_nrfx_config i2s_nrfx_config_##idx = {			    \
		.sck_pin    = DT_PROP(DT_NODELABEL(i2s##idx), sck_pin),			    \
		.lrck_pin   = DT_PROP(DT_NODELABEL(i2s##idx), lrck_pin),			   \
		.mck_pin    = DT_PROP(DT_NODELABEL(i2s##idx), mck_pin),			    \
		.sdout_pin  = DT_PROP(DT_NODELABEL(i2s##idx), sdout_pin),		    \
		.sdin_pin   = DT_PROP(DT_NODELABEL(i2s##idx), sdin_pin),			   \
		.irq_config = i2s_nrfx_irq_##idx##_config,				    \
	};										    \
											    \
	static struct i2s_nrfx_data i2s_nrfx_data_##idx = {				    \
		.state = I2S_IF_NOT_READY,						    \
		.size = 0,								    \
		.tx = {									    \
			.last_trigger_cmd = I2S_TRIGGER_PREPARE,			    \
		},									    \
		.rx = {									    \
			.last_trigger_cmd = I2S_TRIGGER_PREPARE,			    \
		}									    \
	};										    \
											    \
	DEVICE_DT_DEFINE(DT_NODELABEL(i2s##idx),					    \
			 &i2s_nrfx_initialize, device_pm_control_nop, &i2s_nrfx_data_##idx, \
			 &i2s_nrfx_config_##idx, POST_KERNEL,				    \
			 CONFIG_I2S_INIT_PRIORITY, &i2s_nrf_driver_api);		    \
											    \
	static void i2s_nrfx_irq_##idx##_config(const struct device *dev)		    \
	{										    \
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2s##idx)),				    \
			    DT_IRQ(DT_NODELABEL(i2s##idx), priority),			    \
			    nrfx_isr, nrfx_i2s_irq_handler, 0);				    \
		irq_enable(DT_IRQN(DT_NODELABEL(i2s##idx)));				    \
	}


#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2s0), okay)
I2S_NRFX_DEVICE(0);
#endif
