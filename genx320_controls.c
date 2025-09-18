// SPDX-License-Identifier: GPL-2.0-only
#include <media/v4l2-ctrls.h>
#include "genx320_controls.h"
#include "genx320.h"
#include "psee_controls.h"
#include "drivers/genx320/genx320_roi.h"
#include "drivers/genx320/genx320_roi_pixel.h"
#include "drivers/genx320/genx320_erc.h"
#include "drivers/genx320/genx320_bias.h"
#include "drivers/genx320/genx320_mipi.h"
#include "drivers/genx320/genx320_io.h"
#include "drivers/genx320/genx320.h"

static struct psee_roi_master_ops genx320_roi_window_ops = {
	.init = genx320_roi_window_init,
	.append = genx320_roi_window_append,
	.update = genx320_roi_window_update,
	.set = genx320_roi_window_set,
	.reset = genx320_roi_window_reset,
	.enable_roni = genx320_roi_window_enable_roni,
};

static struct psee_roi_pixel_ops genx320_roi_pixel_ops = {
	.init = genx320_roi_pixel_init,
	.set_pixel = genx320_roi_pixel_set_pixel,
	.get_pixel = genx320_roi_pixel_get_pixel,
	.set_array = genx320_roi_pixel_set_array,
	.get_array = genx320_roi_pixel_get_array,
	.reset = genx320_roi_pixel_reset,
};

static struct psee_erc_ops genx320_erc_ops = {
	.init = genx320_erc_init,
	.enable = genx320_erc_enable,
	.s_rate = genx320_erc_set_event_rate,
};

static const struct psee_bias_ops genx320_bias_ops = {
	.init = genx320_bias_init,
	.set = genx320_bias_set,
	.get = genx320_bias_get,
	.get_min = genx320_bias_get_min,
	.get_max = genx320_bias_get_max,
	.get_default = genx320_bias_get_default,
	.get_name = genx320_bias_get_name,
};

static struct psee_esp_ops genx320_esp_ops = {
	.erc = &genx320_erc_ops,
	.bias = &genx320_bias_ops,
	.roi_window = &genx320_roi_window_ops,
	.roi_pixel = &genx320_roi_pixel_ops,
};

static struct psee_mipi_ops genx320_mipi_ops = {
	.configure = genx320_mipi_configure,
};

static struct psee_io_ops genx320_io_ops = {
	.configure_sync_mode = genx320_io_sync_mode,
};

static struct psee_core_ops genx320_core_ops = {
	.start = genx320_start_streaming,
	.stop = genx320_stop_streaming,
	.s_format = genx320_set_event_format,
	.get_width = genx320_get_width,
	.get_height = genx320_get_height,
};

static struct psee_ops genx320_ops = {
	.esp = &genx320_esp_ops,
	.mipi = &genx320_mipi_ops,
	.io = &genx320_io_ops,
	.core = &genx320_core_ops,
};

int genx320_init_controls(struct genx320 *genx320, const struct psee_ctrl_ops *ctrl_ops)
{
	struct v4l2_ctrl_handler *hdl = &genx320->pcw.hdl;

	psee_init_controls(&genx320->pcw, ctrl_ops, &genx320_ops);
	hdl->lock = &genx320->mutex;
	return 0;
}

