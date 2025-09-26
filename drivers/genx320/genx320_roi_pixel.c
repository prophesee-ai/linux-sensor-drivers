// SPDX-License-Identifier: GPL-2.0-only

#include "../common.h"
#include "genx320_registers.h"
#include "genx320_roi_pixel.h"
#include "genx320_roi.h"

#define GENX320_PIXEL_ARRAY_WIDTH 320U
#define GENX320_PIXEL_ARRAY_HEIGHT 320U

static int __apply(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct roi_pixel_config *config = &controls->roi_pixel;

	u32 y, x;
	u32 reg_y_sel;
	u32 reg_val;

	// Iterate over each rows
	for (y = 0; y < config->grid.height; y++)  {
		if (!config->grid.rows[y].dirty)
			continue;

		// Compute roi_y register name and select from row index
		reg_y_sel = y / 32;
		reg_val    = (1U << (y % 32));

		RET_ON(ctrl.write_reg(ctrl.hdl, td_roi_y_array_address + 4 * reg_y_sel, reg_val));

		// Iterate over each columns composed of 32 bits vectors
		for (x = 0; x < (config->grid.width / 32); x++) {
			u32 vector = config->grid.rows[y].vectors[x];

			RET_ON(ctrl.write_reg(ctrl.hdl, td_roi_x_array_address + 4 * x, ~vector));
		}

		// Apply configuration to the hardware by triggering D-Latches toggling of their
		// respective output
		RET_ON(write_field(ctrl, roi_ctrl, td_shadow_trigger, 0));
		RET_ON(write_field(ctrl, roi_ctrl, td_shadow_trigger, 1));

		RET_ON(ctrl.write_reg(ctrl.hdl, td_roi_y_array_address + 4 * reg_y_sel, 0));
		RET_ON(write_field(ctrl, roi_ctrl, td_shadow_trigger, 0));
		RET_ON(write_field(ctrl, roi_ctrl, td_shadow_trigger, 1));

		// reset dirty
		config->grid.rows[y].dirty = false;
	}

	RET_ON(write_field(ctrl, roi_ctrl, td_shadow_trigger, 0));

	return 0;
}

static void __clear(struct psee_controls *controls)
{
	struct roi_pixel_config *config = &controls->roi_pixel;
	u32 y;

	// reset all to enabled, dirty
	for (y = 0; y < config->grid.height; y++)  {
		memset(&config->grid.rows[y].vectors, 0xFF, (config->grid.width / 32));
		// TODO: use roi window to clear ?
		config->grid.rows[y].dirty = false;
	}
}

int genx320_roi_pixel_init(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct roi_pixel_config *config = &controls->roi_pixel;
	roi_ctrl roi_ctrl;

	__clear(controls);

	RET_ON(read_register(ctrl, roi_ctrl, &roi_ctrl.raw));
	roi_ctrl.roi_td_en = 1;
	roi_ctrl.px_iphoto_en = 0;
	roi_ctrl.px_sw_rstn = 1;
	roi_ctrl.td_shadow_trigger = 0;
	RET_ON(write_register(ctrl, roi_ctrl, roi_ctrl.raw));

	config->grid.width = GENX320_PIXEL_ARRAY_WIDTH;
	config->grid.height = GENX320_PIXEL_ARRAY_HEIGHT;

	return 0;
}

int genx320_roi_pixel_reset(struct psee_controls *controls)
{
	__clear(controls);
	RET_ON(__apply(controls));
	return 0;
}

int genx320_roi_pixel_apply(struct psee_controls *controls)
{
	// if enabled or smth like that...
	return __apply(controls);
}

int genx320_roi_pixel_set_pixel(struct psee_controls *controls, u32 x, u32 y, bool enabled)
{
	struct roi_pixel_config *config = &controls->roi_pixel;
	u32 vector_idx, bit_idx, reg_val, mask, saved_fields, write_field, new_reg_val;

	if (x >= GENX320_PIXEL_ARRAY_WIDTH || y >= GENX320_PIXEL_ARRAY_HEIGHT)
		return -EINVAL;

	vector_idx = x / 32;
	bit_idx    = x % 32;
	reg_val    = config->grid.rows[y].vectors[vector_idx];
	mask       = (1 << bit_idx);
	saved_fields = reg_val & (~mask);
	write_field  = (((u32)enabled) << bit_idx);
	new_reg_val  = saved_fields | write_field;
	config->grid.rows[y].dirty = (reg_val != new_reg_val);
	config->grid.rows[y].vectors[vector_idx] = new_reg_val;
	return 0;
}

int genx320_roi_pixel_get_pixel(struct psee_controls *controls, u32 x, u32 y, bool *enabled)
{
	return -EINVAL;
}

int genx320_roi_pixel_set_array(struct psee_controls *controls, struct grid *grid)
{
	u32 y, x;
	struct roi_pixel_config *config = &controls->roi_pixel;
	bool updated = false;

	if (!grid || grid->width != config->grid.width || grid->height != config->grid.height)
		return -EINVAL;

	for (y = 0; y < grid->height; ++y) {
		for (x = 0; x < grid->width / 32; ++x)
			if (config->grid.rows[y].vectors[x] != grid->rows[y].vectors[x]) {
				config->grid.rows[y].vectors[x] = grid->rows[y].vectors[x];
				config->grid.rows[y].dirty = true;
				updated = true;
			}
	}

	if (!updated)
		return 0;

	return __apply(controls);
}

int genx320_roi_pixel_get_array(struct psee_controls *controls, struct grid *grid)
{
	u32 y, x;
	struct roi_pixel_config *config = &controls->roi_pixel;

	if (!grid || grid->width != config->grid.width || grid->height != config->grid.height)
		return -EINVAL;

	for (y = 0; y < grid->height; ++y) {
		for (x = 0; x < grid->width / 32; ++x)
			grid->rows[y].vectors[x] = config->grid.rows[y].vectors[x];
	}
	return 0;
}

int genx320_roi_pixel_update_windows(struct psee_controls *controls, struct roi *rois, u32 n)
{
	u32 i;
	u32 x, y;

	for (i = 0; i < n; i++) {
		for (x = rois[i].x; x < rois[i].width; x++) {
			for (y = rois[i].y; y < rois[i].height; y++)
				genx320_roi_pixel_set_pixel(controls, rois[i].x, rois[i].y, true);
		}
	}
	return 0;
}

