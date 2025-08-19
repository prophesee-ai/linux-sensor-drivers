/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __INCLUDED_GENX32__
#define __INCLUDED_GENX32__

#include <media/v4l2-subdev.h>
#include "psee_controls.h"

#define GENX320_MAX_NUM_SUPPLIES 3

/**
 * struct genx320 - genx320 sensor device structure
 * @pad: Media pad. Only one pad supported
 * @nreset_gpio: Sensor RSTn gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @mutex: Mutex for serializing sensor controls
 * @pcw: wrapper for V4L2 operations
 * @link_freq: frequency of the CSI-2 clock lane
 * @format_code: Media-ctl code of the output format
 */

struct genx320 {
	struct media_pad pad;
	struct gpio_desc *nreset_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[GENX320_MAX_NUM_SUPPLIES];
	struct mutex mutex;
	struct psee_v4l2_ctrl_wrapper pcw;
	struct v4l2_rect crop;

	s64 link_freq;
	u32 format_code;
	u32 rstn_wait_ms;
};

#endif
