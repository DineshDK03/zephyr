/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NRFX_I2S_H_
#define _NRFX_I2S_H_

/* @brief I2S driver clock configuration structure. */
struct i2s_clk_settings_t {
	uint32_t frequency;                     /*< Configured frequency [Hz]. */
	nrf_i2s_ratio_t ratio[3];               /*< Content of CONFIG.RATIO register
						 * for given frequency. Every element
						 * of ratio[3] array corresponds to
						 * one of 3 possible sample width
						 * values
						 * (accordingly 8, 16 and 24 bit).
						 */
	nrf_i2s_mck_t divider[3];               /*< Content of CONFIG.MCKFREQ register
						 * for given frequency. Every element
						 * of divider[3] array corresponds to
						 * one of 3 possible sample width
						 * values
						 *(accordingly 8, 16 and 24 bit).
						 */
};

/**@brief I2S driver clock configfuration table. */
/**
 * The values below are not strict sample rates.
 * They are calculated by formula:
 *
 * Real Sample rate = 32MHz / (DIV * RATIO)
 * ,where
 * DIV = i2s_clk_settings_t.divider[Word length / 8 - 1]
 * RATIO = i2s_clk_settings_t.ratio[Word length / 8 - 1]
 *
 * Please refer to table:
 *
 *------------------------------------------------------------------------
 *|Desired	|Word   |Mode           |Real sample    |Sample         |
 *|sample       |length |               |rate           |rate error     |
 *|rate         |       |               |               |               |
 *------------------------------------------------------------------------
 *|[Hz]         |[bits] |               |[Hz]           |[%]            |
 *------------------------------------------------------------------------
 *|1000         |8      |STEREO         |1000           |0              |
 *|2000         |8      |STEREO         |2000           |0              |
 *|4000         |8      |STEREO         |4000           |0              |
 *|8000         |8      |STEREO         |8000           |0              |
 *|10000        |8      |STEREO         |10416.66667    |4.166666667    |
 *|11025        |8      |STEREO         |11111.11111    |0.781053162    |
 *|12000        |8      |STEREO         |11904.7619     |-0.793650794   |
 *|16000        |8      |STEREO         |15873.01587    |-0.793650794   |
 *|20000        |8      |STEREO         |20833.33333    |4.166666667    |
 *|22050        |8      |STEREO         |22222.22222    |0.781053162    |
 *|24000        |8      |STEREO         |23809.52381    |-0.793650794   |
 *|30000        |8      |STEREO         |30303.0303     |1.01010101     |
 *|32000        |8      |STEREO         |32258.06455    |0.806451613    |
 *|44100        |8      |STEREO         |44444.44444    |0.781053162    |
 *|48000        |8      |STEREO         |47619.03125    |-0.79368489    |
 *########################################################################
 *|1000         |16     |STEREO         |1000           |0              |
 *|2000         |16     |STEREO         |2000           |0              |
 *|4000         |16     |STEREO         |4000           |0              |
 *|8000         |16     |STEREO         |8000           |0              |
 *|10000        |16     |STEREO         |10416.66667    |4.166666667    |
 *|11025        |16     |STEREO         |11111.11111    |0.781053162    |
 *|12000        |16     |STEREO         |11904.7619     |-0.793650794   |
 *|16000        |16     |STEREO         |15873.01587    |-0.793650794   |
 *|20000        |16     |STEREO         |20833.33333    |4.166666667    |
 *|22050        |16     |STEREO         |22222.22222    |0.781053162    |
 *|24000        |16     |STEREO         |23809.52381    |-0.793650794   |
 *|30000        |16     |STEREO         |30303.0303     |1.01010101     |
 *|32000        |16     |STEREO         |32258.06455    |0.806451613    |
 *|44100        |16     |STEREO         |43478.26087    |-1.409839298   |
 *|48000        |16     |STEREO         |47619.03125    |-0.79368489    |
 *########################################################################
 *|1000         |24     |STEREO         |1322.751323    |32.27513228    |
 *|2000         |24     |STEREO         |1984.126984    |-0.793650794   |
 *|4000         |24     |STEREO         |3968.253968    |-0.793650794   |
 *|8000         |24     |STEREO         |7936.507937    |-0.793650794   |
 *|10000        |24     |STEREO         |10416.66667    |4.166666667    |
 *|11025        |24     |STEREO         |11111.11111    |0.781053162    |
 *|12000        |24     |STEREO         |11111.11111    |-7.407407407   |
 *|16000        |24     |STEREO         |15873.01587    |-0.793650794   |
 *|20000        |24     |STEREO         |20833.33333    |4.166666667    |
 *|22050        |24     |STEREO         |22222.22222    |0.781053162    |
 *|24000        |24     |STEREO         |22222.22222    |-7.407407407   |
 *|30000        |24     |STEREO         |30303.0303     |1.01010101     |
 *|32000        |24     |STEREO         |31746.03175    |-0.793650794   |
 *|44100        |24     |STEREO         |44444.44444    |0.781053162    |
 *|48000        |24     |STEREO         |50000          |4.166666666    |
 *------------------------------------------------------------------------
 */

#define NRFX_I2S_AVAILABLE_CLOCK_SETTINGS							\
{												   \
	{											   \
		.frequency = 1000,								   \
		.ratio = { NRF_I2S_RATIO_256X, NRF_I2S_RATIO_256X, NRF_I2S_RATIO_384X },	   \
		.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV63 }, \
	},											   \
	{											   \
		.frequency = 2000,								   \
		.ratio = { NRF_I2S_RATIO_128X, NRF_I2S_RATIO_128X, NRF_I2S_RATIO_384X },	   \
		.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
	},											   \
	{											   \
		.frequency = 4000,								   \
		.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_192X },		   \
		.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
	},											   \
	{											   \
		.frequency = 8000,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
	},											   \
	{											   \
		.frequency = 10000,								   \
		.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_192X },		   \
		.divider = { NRF_I2S_MCK_32MDIV32, NRF_I2S_MCK_32MDIV32, NRF_I2S_MCK_32MDIV16 },   \
	},											   \
	{											   \
		.frequency = 11025,								   \
		.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_192X },		   \
		.divider = { NRF_I2S_MCK_32MDIV30, NRF_I2S_MCK_32MDIV30, NRF_I2S_MCK_32MDIV15 },   \
	},											   \
	{											   \
		.frequency = 12000,								   \
		.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_192X },		   \
		.divider = { NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV15 },   \
	},											   \
	{											   \
		.frequency = 16000,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV63, NRF_I2S_MCK_32MDIV63, NRF_I2S_MCK_32MDIV21 },   \
	},											   \
	{											   \
		.frequency = 20000,								   \
		.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV16, NRF_I2S_MCK_32MDIV16, NRF_I2S_MCK_32MDIV16 },   \
	},											   \
	{											   \
		.frequency = 22050,								   \
		.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV15 },   \
	},											   \
	{											   \
		.frequency = 24000,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV15 },   \
	},											   \
	{											   \
		.frequency = 30000,								   \
		.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X },		   \
		.divider = { NRF_I2S_MCK_32MDIV11, NRF_I2S_MCK_32MDIV11, NRF_I2S_MCK_32MDIV11 },   \
	},											   \
	{											   \
		.frequency = 32000,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_48X },		   \
		.divider = { NRF_I2S_MCK_32MDIV31, NRF_I2S_MCK_32MDIV31, NRF_I2S_MCK_32MDIV21 },   \
	},											   \
	{											   \
		.frequency = 44100,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_48X },		   \
		.divider = { NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV15 },   \
	},											   \
	{											   \
		.frequency = 48000,								   \
		.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_64X },		   \
		.divider = { NRF_I2S_MCK_32MDIV21, NRF_I2S_MCK_32MDIV21, NRF_I2S_MCK_32MDIV10 },   \
	},											   \
}

enum i2s_if_state {
	I2S_IF_NOT_READY = 0,
	I2S_IF_READY,
	I2S_IF_RESTARTING,
	I2S_IF_RUNNING,
	I2S_IF_STOPPING,
	I2S_IF_NEEDS_RESTART,
	I2S_IF_ERROR
};

struct i2s_nrfx_config {
	uint8_t sck_pin;
	uint8_t lrck_pin;
	uint8_t mck_pin;
	uint8_t sdout_pin;
	uint8_t sdin_pin;
	void (*irq_config)(const struct device *dev);
};

struct stream {
	int32_t state;
	struct k_sem sem;
	struct k_mem_slab *mem_slab;
	int32_t timeout;

	struct k_msgq queue;
	void *rx_msgs[CONFIG_NRFX_I2S_RX_BLOCK_COUNT];
	void *tx_msgs[CONFIG_NRFX_I2S_TX_BLOCK_COUNT];
	enum i2s_trigger_cmd last_trigger_cmd;
	struct i2s_config config;
};

struct i2s_nrfx_data {
	enum i2s_if_state state;
	size_t size;
	nrfx_i2s_buffers_t buffers;

	struct k_work  i2s_work;
	nrfx_i2s_buffers_t const *p_released;
	uint32_t status;

	struct stream tx;
	struct stream rx;
};

#endif  /* _NRFX_I2S_H_ */
