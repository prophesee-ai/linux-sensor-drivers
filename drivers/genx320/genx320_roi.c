// SPDX-License-Identifier: GPL-2.0-only
#include "../common.h"
#include "genx320_registers.h"
#include "genx320_roi.h"
#include "genx320_roi_pixel.h"

static int __set_window(struct psee_controls *controls, struct roi roi, u32 index)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	u32 xaddr = roi_win_array_address + 8 * index;
	u32 yaddr = xaddr + 4;
	roi_win_array roi_win;

	roi_win.roi_win_start = roi.x;
	roi_win.roi_win_end = roi.x + roi.width;
	RET_ON(ctrl.write_reg(ctrl.hdl, xaddr, roi_win.raw));

	roi_win.roi_win_start = roi.y;
	roi_win.roi_win_end = roi.y + roi.height;
	RET_ON(ctrl.write_reg(ctrl.hdl, yaddr, roi_win.raw));
	return 0;
}

static int __apply(struct psee_controls *controls, u32 n, enum RoiMode mode)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	u32 master_en, master_done = 0, master_busy = 1, timeout = 50;

	read_field(ctrl, roi_master_ctrl, master_en, &master_en);
	if (!master_en)
		return -EBUSY;

	RET_ON(write_field(ctrl, roi_ctrl, px_roi_halt_programming, 0));

	RET_ON(write_fields(ctrl, roi_master_ctrl, {
		__s(master_run, 1)
		__s(master_mode, !!mode)
		__s(win_nb, n)
	}));

	while (master_busy && (timeout-- > 0))
		RET_ON(read_field(ctrl, roi_master_ctrl, master_busy, &master_busy));

	if (timeout <= 0)
		return -ETIMEDOUT;

	timeout = 50;
	if (!master_busy)
		while ((!master_done) && (timeout-- > 0))
			RET_ON(read_field(ctrl, roi_master_ctrl, master_done, &master_done));

	return master_done ? 0 : -ETIMEDOUT;
}

int apply(struct psee_controls *controls)
{
	struct roi_window_config *config = &controls->roi_window;

	if (config->roi_count)
		return __apply(controls, config->roi_count, config->mode);
	return 0;
}

int genx320_roi_window_enable(struct psee_controls *controls, bool en)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	roi_win_array saved_x;
	roi_win_array saved_y;

	if (en)
		// just reapply the current config
		return apply(controls);

	RET_ON(ctrl.read_reg(ctrl.hdl, roi_win_array_address, &saved_x.raw));
	RET_ON(ctrl.read_reg(ctrl.hdl, roi_win_array_address + 4, &saved_y.raw));

	RET_ON(__set_window(controls, (struct roi) {0, 0, 320, 320}, 0));

	RET_ON(__apply(controls, 1, ROI));

	return __set_window(controls, (struct roi) {
		.x      = saved_x.roi_win_start,
		.width  = saved_x.roi_win_end - saved_x.roi_win_start,
		.y      = saved_y.roi_win_start,
		.height = saved_y.roi_win_end - saved_y.roi_win_start,
	}, 1);
}

int genx320_roi_window_enable_roni(struct psee_controls *controls, bool en)
{
	struct roi_window_config *config = &controls->roi_window;
	enum RoiMode mode = en ? RONI : ROI;

	if (config->mode != mode) {
		config->mode = mode;
		apply(controls);
	}

	return 0;
}

int genx320_roi_window_reset(struct psee_controls *controls)
{
	struct roi_window_config *config = &controls->roi_window;

	RET_ON(__set_window(controls, (struct roi) {0, 0, 320, 320}, 0));

	RET_ON(__apply(controls, 1, ROI));

	config->roi_count = 0;
	return 0;
}

int genx320_roi_window_init(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct roi_window_config *config = &controls->roi_window;

	config->roi_count = 0;
	config->mode = 0;

	RET_ON(write_fields(ctrl, roi_ctrl, {
		__s(roi_td_en, 1)
		__s(px_iphoto_en, 0)
		__s(px_sw_rstn, 1)
		__s(td_shadow_trigger, 0)
	}));

	RET_ON(write_field(ctrl, roi_master_chicken_bit, driver_register_if_en, 0));
	RET_ON(write_field(ctrl, roi_ctrl, px_roi_halt_programming, 0));
	RET_ON(write_fields(ctrl, roi_master_ctrl, {
		__s(master_en, 1)
		__s(master_run, 0)
	}));

	RET_ON(genx320_roi_window_reset(controls));

	return 0;
}

int genx320_roi_window_append(struct psee_controls *controls, struct roi roi, bool single)
{
	struct roi_window_config *config = &controls->roi_window;

	if (config->roi_count >= MAX_ROI_WINDOWS)
		return -EINVAL;

	if (roi.width == 0 || roi.height == 0)
		return 0;

	RET_ON(__set_window(controls, roi, config->roi_count));

	config->roi_count++;

	return single ? apply(controls) : 0;
}

int genx320_roi_window_update(struct psee_controls *controls, struct roi roi, u32 index)
{
	struct roi_window_config *config = &controls->roi_window;

	if (config->roi_count && index >= config->roi_count)
		return -EINVAL;

	RET_ON(__set_window(controls, roi, index));

	return apply(controls);
}

int genx320_roi_window_set(struct psee_controls *controls, struct roi *rois, u32 n)
{
	int i;
	struct roi_window_config *config = &controls->roi_window;

	if (n > MAX_ROI_WINDOWS)
		return -EINVAL;

	if (n == 0) {
		RET_ON(genx320_roi_window_reset(controls));
		return 0;
	}

	config->roi_count = 0;
	for (i = 0; i < n; i++)
		RET_ON(genx320_roi_window_append(controls, rois[i], false));

	return apply(controls);
}
