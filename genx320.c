// SPDX-License-Identifier: GPL-2.0-only
/*
 * Prophesee genx320 Camera Sensor Driver
 *
 * Copyright (C) 2023 Prophesee
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "psee_controls.h"
#include "genx320_controls.h"
#include "genx320.h"
#include "drivers/genx320/genx320_registers.h"
#include "psee-format.h"

#define PIXEL_ARRAY_WIDTH 320
#define PIXEL_ARRAY_HEIGHT 320

#define GENX320_NUM_DATA_LANES 1
#define GENX320_INCLK_RATE 20000000

/*
 * Sensor registers
 */

#define GENX320_CHIP_ID 0x14
#define GENX320_ID 0xb0602003

#define GENX320_BIAS_BASE 0x1000
#define BIAS_PR_HV0 (GENX320_BIAS_BASE + 0x000)
#define BIAS_FO_HV0 (GENX320_BIAS_BASE + 0x004)
#define BIAS_FES_HV0 (GENX320_BIAS_BASE + 0x008)
#define BIAS_HPF_LV0 (GENX320_BIAS_BASE + 0x100)
#define BIAS_DIFF_ON_LV0 (GENX320_BIAS_BASE + 0x104)
#define BIAS_DIFF_LV0 (GENX320_BIAS_BASE + 0x108)
#define BIAS_DIFF_OFF_LV0 (GENX320_BIAS_BASE + 0x10C)
#define BIAS_INV_LV0 (GENX320_BIAS_BASE + 0x110)
#define BIAS_REFR_LV0 (GENX320_BIAS_BASE + 0x114)
#define BIAS_INVP_LV0 (GENX320_BIAS_BASE + 0x118)
#define BIAS_REQ_PU_LV0 (GENX320_BIAS_BASE + 0x11C)
#define BIAS_SM_PDY_LV0 (GENX320_BIAS_BASE + 0x120)

/* MBX registers */
#define MBX_BASE 0xF000

#define GENX320_MBX_MISC (MBX_BASE + 0x0010)
#define GENX320_BOOT_MAGIC 3405691582u

static const char * const genx320_supply_names[] = {
	"vadd",		/* Supply voltage (Analog) */
	"vddd1",	/* Supply voltage (Digital 1) */
	"vddd2",	/* Supply voltage (Digital 2) */
};

static const s64 link_freq[] = {
	750000000,
};

/* Supported sensor media formats */
static const u32 supported_formats[] = {
	MEDIA_BUS_FMT_PSEE_EVT2,
	MEDIA_BUS_FMT_PSEE_EVT3,
	MEDIA_BUS_FMT_PSEE_EVT21,
};

/**
 * to_genx320() - genx320 V4L2 sub-device to genx320 device.
 * @subdev: pointer to genx320 V4L2 sub-device
 *
 * Return: pointer to genx320 device
 */
static inline struct genx320 *to_genx320(struct v4l2_subdev *subdev)
{
	struct psee_v4l2_ctrl_wrapper *pcw = sd_to_pcw(subdev);

	return container_of(pcw, struct genx320, pcw);
}

/**
 * genx320_read() - Read registers.
 * @genx320: pointer to genx320 device
 * @reg: register address
 * @val: pointer to register array to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_read(struct genx320 *genx320, u32 reg, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&genx320->pcw.sd);
	struct i2c_msg xfer[2] = {0};
	int ret;

	xfer[0].addr = client->addr;
	reg = cpu_to_be16((u16)reg);
	xfer[0].buf = (u8 *)&reg;
	xfer[0].len = 2; // genx320 registers are 16bits long
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = (u8 *)val;
	xfer[1].len = sizeof(*val);

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret != 2) {
		dev_warn(genx320->pcw.dev, "read ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	} else {
		*val = be32_to_cpu(*val);
		ret = 0;
	}

	return ret;
}

static int genx320_ctrl_read(void *hdl, u32 reg, u32 *val)
{
	struct psee_v4l2_ctrl_wrapper *pcw = hdl;
	struct genx320 *genx320 = container_of(pcw, struct genx320, pcw);

	return genx320_read(genx320, reg, val);
}

/**
 * genx320_write_reg() - Write one register
 * @genx320: pointer to genx320 device
 * @reg: register address
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_write(struct genx320 *genx320, u32 reg, const u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&genx320->pcw.sd);
	struct i2c_msg xfer = {0};
	u8 buf[2 + 4] = {0};
	u16 *regp = (u16 *)&buf[0];
	u32 *valp = (u32 *)&buf[2];
	int ret;

	xfer.addr = client->addr;
	*regp = cpu_to_be16((u16)reg);
	*valp = cpu_to_be32(val);

	xfer.buf = buf;
	xfer.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret > 0) {
		ret = 0;
	} else {
		dev_warn(genx320->pcw.dev, "write ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	return ret;
}

static int genx320_ctrl_write(void *hdl, u32 reg, u32 val)
{
	struct psee_v4l2_ctrl_wrapper *pcw = hdl;
	struct genx320 *genx320 = container_of(pcw, struct genx320, pcw);

	return genx320_write(genx320, reg, val);
}

static int genx320_set(struct genx320 *genx320, u32 reg, const u32 mask)
{
	u32 value;

	RET_ON(genx320_read(genx320, reg, &value));
	value |= mask;
	RET_ON(genx320_write(genx320, reg, value));
	return 0;
}

static int genx320_clear(struct genx320 *genx320, u32 reg, const u32 mask)
{
	u32 value;

	RET_ON(genx320_read(genx320, reg, &value));
	value &= ~(mask);
	RET_ON(genx320_write(genx320, reg, value));
	return 0;
}

/**
 * genx320_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to genx320 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
#ifdef OMIT_PSEE_FORMATS 
	 if (code->index > 0)
        return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y8_1X8;
#else
	if (code->index >= ARRAY_SIZE(supported_formats))
		return -EINVAL;

	code->code = supported_formats[code->index];
#endif
	return 0;
}

/**
 * genx320_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to genx320 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index != 0)
		return -EINVAL;

	fsize->min_width = 4096;
	fsize->max_width = fsize->min_width;
	fsize->min_height = 391;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * genx320_fill_pad_format() - Fill subdevice pad format
 *                            from selected media format
 * @genx320: pointer to genx320 device
 * @code: media-ctl format code in use
 * @fmt: V4L2 sub-device format need to be filled
 */
static void genx320_fill_pad_format(struct genx320 *genx320,
				    u32 code,
				    struct v4l2_subdev_format *fmt)
{
	fmt->format.width = 4096;
	fmt->format.height = 391;
	fmt->format.code = code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * genx320_get_pad_format() - Get subdevice pad format
 * @sd: pointer to genx320 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct genx320 *genx320 = to_genx320(sd);

	mutex_lock(&genx320->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
#ifdef OMIT_PSEE_FORMATS 
		genx320_fill_pad_format(genx320, MEDIA_BUS_FMT_Y8_1X8, fmt);
#else	
		genx320_fill_pad_format(genx320, genx320->format_code, fmt);
#endif
	}

	mutex_unlock(&genx320->mutex);

	return 0;
}

/**
 * genx320_set_pad_format() - Set subdevice pad format
 * @sd: pointer to genx320 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct genx320 *genx320 = to_genx320(sd);
	u32 code;
	int ret = 0;

	mutex_lock(&genx320->mutex);

#ifdef OMIT_PSEE_FORMATS 
	code = MEDIA_BUS_FMT_Y8_1X8;
#else
	switch (fmt->format.code) {
	case MEDIA_BUS_FMT_PSEE_EVT3:
		code = MEDIA_BUS_FMT_PSEE_EVT3;
		break;
	case MEDIA_BUS_FMT_PSEE_EVT2:
		code = MEDIA_BUS_FMT_PSEE_EVT2;
		break;
	case MEDIA_BUS_FMT_PSEE_EVT21:
	case MEDIA_BUS_FMT_PSEE_EVT21ME:
	default:
		code = MEDIA_BUS_FMT_PSEE_EVT21;
		break;
	}
#endif
	genx320_fill_pad_format(genx320, code, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
#ifndef OMIT_PSEE_FORMATS
		genx320->format_code = code;
#endif
	}
	
	mutex_unlock(&genx320->mutex);

	return ret;
}

/**
 * genx320_init_pad_cfg() - Initialize sub-device pad configuration
 * @sd: pointer to genx320 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_init_pad_cfg(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state)
{
	struct genx320 *genx320 = to_genx320(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	genx320_fill_pad_format(genx320, genx320->format_code, &fmt);

	return genx320_set_pad_format(sd, sd_state, &fmt);
}

static struct v4l2_rect *
genx320_get_pad_crop(struct genx320 *genx320,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &genx320->crop;
	}

	return NULL;
}

static int genx320_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct genx320 *genx320 = to_genx320(sd);
	struct v4l2_rect *crop;
	struct roi roi;
	int ret = 0;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	mutex_lock(&genx320->mutex);
	crop = genx320_get_pad_crop(genx320, sd_state, sel->pad, sel->which);
	crop->left = clamp(sel->r.left, 0, PIXEL_ARRAY_WIDTH - 1);
	crop->top = clamp(sel->r.top, 0, PIXEL_ARRAY_HEIGHT - 1);
	crop->width = clamp((s32)sel->r.width, 1, PIXEL_ARRAY_WIDTH - crop->left);
	crop->height = clamp((s32)sel->r.height, 1, PIXEL_ARRAY_HEIGHT - crop->top);

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		roi.x = crop->left;
		roi.y = crop->top;
		roi.width = crop->width;
		roi.height = crop->height;
		ret = call_esp_op(&genx320->pcw, roi_window, set, &roi, 1);
	}

	mutex_unlock(&genx320->mutex);

	return ret;
}


static int genx320_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct genx320 *genx320 = to_genx320(sd);

		mutex_lock(&genx320->mutex);
		sel->r = *genx320_get_pad_crop(genx320, sd_state, sel->pad,
		sel->which);
		mutex_unlock(&genx320->mutex);
		return 0;
	}

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = PIXEL_ARRAY_WIDTH;
		sel->r.height = PIXEL_ARRAY_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

/**
 * genx320_reconfigure_csi2_freq() - Reconfigure the clock tree for the selected CSI-2 freq
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static  __maybe_unused int genx320_reconfigure_csi2_freq(struct genx320 *genx320)
{
	sys_clk_ctrl sys_clk_ctrl;

	// pll_powerdown_and_switch
	RET_ON(genx320_read(genx320, sys_clk_ctrl_address, &sys_clk_ctrl.raw));

	sys_clk_ctrl.sys_clk_en = 1;
	sys_clk_ctrl.sys_clk_switch = 0;
	RET_ON(genx320_write(genx320, sys_clk_ctrl_address, sys_clk_ctrl.raw));

	sys_clk_ctrl.sys_clk_en = 0;
	sys_clk_ctrl.sys_clk_switch = 0;

	RET_ON(genx320_write(genx320, sys_clk_ctrl_address, sys_clk_ctrl.raw));
	RET_ON(genx320_write(genx320, pll_ctrl_address, 1));

	//  set_sensor_clk_freq
	return 0;
}

static int genx320_configure_mipi(struct genx320 *genx320)
{
	struct mipi_config *mipi = &genx320->pcw.controls.mipi;

	// TODO: get configuration from device tree
	mipi->format = VARIABLE_SIZE;
	mipi->bit_rate = 800;
	mipi->num_lanes = 1;
	mipi->stats_en = true;
	mipi->eof_marker = true;

	RET_ON(call_mipi_op(&genx320->pcw, configure));
	return 0;
}

/**
 * genx320_apply_format() - Set the sensor to output the selected format
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_apply_format(struct genx320 *genx320)
{
	enum event_format format;
	switch (genx320->format_code) {
	case MEDIA_BUS_FMT_PSEE_EVT2:
		format = EVENT_FORMAT_EVT2;
		break;
	case MEDIA_BUS_FMT_PSEE_EVT3:
		format = EVENT_FORMAT_EVT3;
		break;
	case MEDIA_BUS_FMT_PSEE_EVT21:
		format = EVENT_FORMAT_EVT21;
		break;
	default:
		return -EINVAL;
	}

	call_core_op(&genx320->pcw, s_format, format);
	return 0;
}

/**
 * genx320_check_boot() - Check the boot magic
 * @genx320: pointer to the genx320 device
 *
 * There is a misc register switching to a magic value at the end of sensor boot process
 * It should be covered by the wait time given by Prophesee, thus this is just a sanity check
 */
static int genx320_check_boot(struct genx320 *genx320)
{
	int ret;
	u32 val;

	ret = genx320_read(genx320, GENX320_MBX_MISC, &val);
	if (ret) {
		dev_warn(genx320->pcw.dev, "could not get the boot magic");
		return ret;
	}

	if (val != GENX320_BOOT_MAGIC) {
		dev_warn(genx320->pcw.dev, "unexpected boot magic, got %u, expected %u",
			 val, GENX320_BOOT_MAGIC);
		return -ENXIO;
	}

	return 0;
}

/**
 * genx320_tune_analog() - Update factory settings
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_tune_analog(struct genx320 *genx320)
{
	bgen bias_value = { .raw = 0 };
	bgen_ctrl bgen_ctrl = { .raw = 0 };

	bgen_ctrl.bias_rstn_hv = 1;
	bgen_ctrl.bias_rstn_lv = 1;
	RET_ON(genx320_set(genx320, bgen_ctrl_address, bgen_ctrl.raw));
	usleep_range(200, 250);

	// configure bias_diff on and off as vdac type biases
	bias_value.ibtype_sel = 1;
	RET_ON(genx320_clear(genx320, BIAS_DIFF_ON_LV0, bias_value.raw));
	RET_ON(genx320_clear(genx320, BIAS_DIFF_OFF_LV0, bias_value.raw));

	// set the post reset bias configuration
	RET_ON(genx320_write(genx320, BIAS_PR_HV0, 0x0301003D));
	RET_ON(genx320_write(genx320, BIAS_FO_HV0, 0x03010022));
	RET_ON(genx320_write(genx320, BIAS_FES_HV0, 0x0101003F));
	RET_ON(genx320_write(genx320, BIAS_HPF_LV0, 0x03010028));
	RET_ON(genx320_write(genx320, BIAS_DIFF_ON_LV0, 0x01010019));
	RET_ON(genx320_write(genx320, BIAS_DIFF_LV0, 0x01010033));
	RET_ON(genx320_write(genx320, BIAS_DIFF_OFF_LV0, 0x0101001C));
	RET_ON(genx320_write(genx320, BIAS_INV_LV0, 0x01010039));
	RET_ON(genx320_write(genx320, BIAS_REFR_LV0, 0x0309000A));
	RET_ON(genx320_write(genx320, BIAS_INVP_LV0, 0x03010038));
	RET_ON(genx320_write(genx320, BIAS_REQ_PU_LV0, 0x03000074));
	RET_ON(genx320_write(genx320, BIAS_SM_PDY_LV0, 0x010000A4));

	bgen_ctrl.burst_transfer_hv_bank_0 = 1;
	bgen_ctrl.burst_transfer_lv_bank_0 = 1;
	return genx320_write(genx320, bgen_ctrl_address, bgen_ctrl.raw);
}

/**
 * genx320_init() - Set sensor ready to stream
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_init(struct genx320 *genx320)
{
	int ret = 0;
	struct core_config *core = &genx320->pcw.controls.core;
	struct psee_v4l2_ctrl_wrapper *pcw = &genx320->pcw;

	// default config
	core->source = SENSOR_SOURCE_PIXEL_ARRAY;
	core->sync_mode = SYNC_MODE_STANDALONE;
	core->sensor_if = SENSOR_IF_MIPI;
	core->format = EVENT_FORMAT_EVT3;

	RET_ON(genx320_configure_mipi(genx320));
	RET_ON(genx320_apply_format(genx320));
	RET_ON(genx320_tune_analog(genx320));

	ret = call_esp_op(pcw, roi_window, init);
	if (ret < 0) {
		dev_err(genx320->pcw.dev, "genx320_roi_window_init failed (%d)\n", ret);
		return ret;
	}

	ret = call_esp_op(pcw, roi_pixel, init);
	if (ret < 0) {
		dev_err(genx320->pcw.dev, "genx320_roi_pixel_init failed (%d)\n", ret);
		return ret;
	}

	ret = call_esp_op(pcw, erc, init);
	if (ret < 0) {
		dev_err(genx320->pcw.dev, "genx320_erc_init failed (%d)\n", ret);
		return ret;
	}

	ret = call_esp_op(pcw, bias, init);
	if (ret < 0) {
		dev_err(genx320->pcw.dev, "genx320_bias_init failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 * genx320_start_streaming() - Start sensor stream
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_start_streaming(struct genx320 *genx320)
{
	struct psee_v4l2_ctrl_wrapper *pcw = &genx320->pcw;
	int ret = 0;


	ret = __v4l2_ctrl_handler_setup(&genx320->pcw.hdl);
	if (ret < 0) {
		dev_err(genx320->pcw.sd.dev, "%s control init failed (%d)\n", __func__, ret);
		return ret;
	}

	return call_core_op(&genx320->pcw, start);
}

/**
 * genx320_stop_streaming() - Stop sensor stream
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_stop_streaming(struct genx320 *genx320)
{
	ro_td_ctrl ro_td_ctrl;
	ro_lp_ctrl ro_lp_ctrl;
	ro_time_base_ctrl ro_time_base_ctrl;
	mipi_csi_ctrl mipi_csi_ctrl;

	RET_ON(genx320_read(genx320, ro_td_ctrl_address, &ro_td_ctrl.raw));
	ro_td_ctrl.ro_td_ack_y_rstn = 0;
	ro_td_ctrl.ro_td_arb_y_rstn = 0;
	ro_td_ctrl.ro_td_addr_y_rstn = 0;
	ro_td_ctrl.ro_td_sendreq_y_rstn = 0;
	RET_ON(genx320_write(genx320, ro_td_ctrl_address, ro_td_ctrl.raw));

	RET_ON(genx320_read(genx320, ro_lp_ctrl_address, &ro_lp_ctrl.raw));
	ro_lp_ctrl.lp_output_disable = 1;
	ro_lp_ctrl.lp_keep_th = 0;
	RET_ON(genx320_write(genx320, ro_lp_ctrl_address, ro_lp_ctrl.raw));

	msleep(1);

	RET_ON(genx320_read(genx320, ro_time_base_ctrl_address, &ro_time_base_ctrl.raw));
	ro_time_base_ctrl.time_base_enable = 0;
	RET_ON(genx320_write(genx320, ro_time_base_ctrl_address, ro_time_base_ctrl.raw));

	RET_ON(genx320_read(genx320, mipi_csi_ctrl_address, &mipi_csi_ctrl.raw));
	mipi_csi_ctrl.enable = 0;
	RET_ON(genx320_write(genx320, mipi_csi_ctrl_address, mipi_csi_ctrl.raw));

	return 0;
}

/**
 * genx320_set_stream() - Enable sensor streaming
 * @sd: pointer to genx320 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct genx320 *genx320 = to_genx320(sd);
	int ret;

	mutex_lock(&genx320->mutex);

	if (genx320->pcw.streaming == enable) {
		mutex_unlock(&genx320->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(genx320->pcw.dev);
		if (ret < 0)
			goto error_unlock;

		ret = genx320_start_streaming(genx320);
		if (ret)
			goto error_power_off;
	} else {
		genx320_stop_streaming(genx320);
		pm_runtime_put(genx320->pcw.dev);
	}

	genx320->pcw.streaming = enable;

	mutex_unlock(&genx320->mutex);

	return 0;

error_power_off:
	pm_runtime_put(genx320->pcw.dev);
error_unlock:
	mutex_unlock(&genx320->mutex);

	return ret;
}

/**
 * genx320_detect() - Detect genx320 sensor
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int genx320_detect(struct genx320 *genx320)
{
	int ret;
	u32 val;

	ret = genx320_read(genx320, GENX320_CHIP_ID, &val);
	if (ret)
		return ret;

	if (val != GENX320_ID) {
		dev_err(genx320->pcw.dev, "chip id mismatch: %x!=%x",
			GENX320_ID, val);
		return -ENXIO;
	}

	return 0;
}

/**
 * genx320_parse_hw_config() - Parse HW configuration and check if supported
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_parse_hw_config(struct genx320 *genx320)
{
	struct fwnode_handle *fwnode = dev_fwnode(genx320->pcw.dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	unsigned long rate;
	unsigned int i, j;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	genx320->nreset_gpio = devm_gpiod_get_optional(genx320->pcw.dev, "nreset",
						       GPIOD_OUT_LOW);
	if (IS_ERR(genx320->nreset_gpio)) {
		dev_err(genx320->pcw.dev, "failed to get reset gpio %ld",
			PTR_ERR(genx320->nreset_gpio));
		return PTR_ERR(genx320->nreset_gpio);
	}

	/* Get sensor input clock */
	genx320->inclk = devm_clk_get(genx320->pcw.dev, NULL);
	if (IS_ERR(genx320->inclk)) {
		dev_dbg(genx320->pcw.dev, "could not get inclk");
		return PTR_ERR(genx320->inclk);
	}

	rate = clk_get_rate(genx320->inclk);
	if (rate != GENX320_INCLK_RATE) {
		dev_err(genx320->pcw.dev, "inclk frequency mismatch");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(genx320_supply_names); i++)
		genx320->supplies[i].supply = genx320_supply_names[i];

	ret = devm_regulator_bulk_get(genx320->pcw.dev,
				      ARRAY_SIZE(genx320_supply_names),
				      genx320->supplies);
	if (ret)
		return ret;

	/* Get hardware dependent timing parameters*/
	genx320->rstn_wait_ms = 55; // CCAM5 introduces 48ms +/-15% delay
	// CCAM5 handling RSTn needs 285 = 55 + 230  (~200 ms delay for reset -> RSTn)
	if (fwnode_property_read_u32(fwnode, "rstn-delay-ms", &genx320->rstn_wait_ms)) {
        dev_warn(genx320->pcw.dev, "Failed to read rstn-delay-ms. Set it if you use the CCAM5 adapter RSTn");
	}

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GENX320_NUM_DATA_LANES) {
		dev_err(genx320->pcw.dev,
			"number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(genx320->pcw.dev, "no link frequencies defined");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(link_freq); j++) {
			if (bus_cfg.link_frequencies[i] == link_freq[j]) {
				dev_info(genx320->pcw.dev, "Using CSI-2 freq %lld", link_freq[j]);
				genx320->link_freq = link_freq[j];
				goto done_endpoint_free;
			}
		}
	}

	dev_err(genx320->pcw.dev, "none of the link frequencies is supported");
	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int genx320_log_status(struct v4l2_subdev *sd)
{
	u32 val;
	struct genx320 *genx320 = to_genx320(sd);
	struct device *dev = genx320->pcw.dev;
	struct mipi_config *mipi = &genx320->pcw.controls.mipi;

	if (mipi->stats_en == false) {
		dev_info(dev, "MIPI_CSI stats not enabled");
		return 0;
	}

	dev_info(dev, "******* MIPI_CSI STATUS ********");
	RET_ON(genx320_read(genx320, mipi_csi_stat_frame_cnt_address, &val));
	dev_info(dev, "Frame Count: %u", val);

	RET_ON(genx320_read(genx320, mipi_csi_stat_byte_cnt_address, &val));
	dev_info(dev, "Byte Count: %u", val);

	RET_ON(genx320_read(genx320, mipi_csi_stat_pad_cnt_address, &val));
	dev_info(dev, "Pad Count: %u", val);

	RET_ON(genx320_read(genx320, mipi_csi_stat_pkt_cnt_address, &val));
	dev_info(dev, "Packet Count: %u", val);

	RET_ON(genx320_read(genx320, mipi_csi_stat_inc_pkt_cnt_address, &val));
	dev_info(dev, "Incomplete Packet Count: %u", val);

	RET_ON(genx320_read(genx320, mipi_csi_stat_frame_period_address, &val));
	dev_info(dev, "Frame Period: %u", val);
	return 0;
}
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int genx320_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct genx320 *genx320 = to_genx320(sd);
	u32 val;
	int ret;

	if ((reg->reg % 4) || reg->reg > 0xFFF0)
		return -EINVAL;

	ret = genx320_read(genx320, (u16)reg->reg, &val);
	reg->val = val;
	reg->size = 4;

	return ret;
}

static int genx320_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct genx320 *genx320 = to_genx320(sd);

	if ((reg->reg % 4) || reg->reg > 0xFFF0)
		return -EINVAL;

	return genx320_write(genx320, (u16)reg->reg, (u32)reg->val);
}
#endif /* def CONFIG_VIDEO_ADV_DEBUG */

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops genx320_video_ops = {
	.s_stream = genx320_set_stream,
};

static const struct v4l2_subdev_core_ops genx320_core_ops = {
	.log_status = genx320_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = genx320_g_register,
	.s_register = genx320_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops genx320_pad_ops = {
	.enum_mbus_code = genx320_enum_mbus_code,
	.enum_frame_size = genx320_enum_frame_size,
	.get_fmt = genx320_get_pad_format,
	.set_fmt = genx320_set_pad_format,
	.get_selection = genx320_get_selection,
	.set_selection = genx320_set_selection,
};

static const struct v4l2_subdev_ops genx320_subdev_ops = {
	.video = &genx320_video_ops,
	.pad = &genx320_pad_ops,
	.core = &genx320_core_ops,
};

static const struct v4l2_subdev_internal_ops genx320_internal_ops = {
	.init_state = genx320_init_pad_cfg,
};

/**
 * genx320_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct genx320 *genx320 = to_genx320(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(genx320_supply_names),
				    genx320->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators");
		return ret;
	}

	ret = clk_prepare_enable(genx320->inclk);
	if (ret) {
		dev_err(genx320->pcw.dev, "fail to enable inclk");
		goto error_reset;
	}

	/* Trstn_start = 200ns (min) */
	usleep_range(1, 100);
	gpiod_set_value_cansleep(genx320->nreset_gpio, 1);

	/* Tstart = 15ms (min) */
	/* but CCAM5 introduces 48ms +/-15% delay */
	msleep_interruptible(15 + genx320->rstn_wait_ms);
	ret = genx320_check_boot(genx320);
	if (ret) {
		dev_err(genx320->pcw.dev, "fail to boot sensor");
		goto error_reset;
	}

	ret = genx320_init(genx320);
	if (ret) {
		dev_err(genx320->pcw.dev, "fail to initialize sensor");
		goto error_reset;
	}

	genx320->pcw.initialized = true;

	return 0;

error_reset:
	gpiod_set_value_cansleep(genx320->nreset_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(genx320_supply_names),
			       genx320->supplies);

	return ret;
}

/**
 * genx320_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct genx320 *genx320 = to_genx320(sd);

	/* Tstopwait = 15ms (min) */
	msleep_interruptible(15);

	gpiod_set_value_cansleep(genx320->nreset_gpio, 0);
	/* Tclk_pd = 200ns (min) */
	usleep_range(1, 100);

	clk_disable_unprepare(genx320->inclk);

	regulator_bulk_disable(ARRAY_SIZE(genx320_supply_names),
			       genx320->supplies);

	genx320->pcw.initialized = false;

	return 0;
}

static const struct psee_ctrl_ops genx320_ctrl_ops = {
	.read_reg = &genx320_ctrl_read,
	.write_reg = &genx320_ctrl_write,
};

#ifdef OMIT_PSEE_FORMATS
static int evt_format_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);
	struct core_config *core = &pcw->controls.core;
	struct genx320 *genx320 = to_genx320(&pcw->sd);

	if (pcw->streaming) {
		dev_err(pcw->sd.dev, "Cannot change format while streaming\n");
		return -EBUSY;
	}
	switch (ctrl->val) {
	case 0:
		core->format = EVENT_FORMAT_EVT2;
		genx320->format_code = MEDIA_BUS_FMT_PSEE_EVT2;
		break;
	case 1:
		core->format = EVENT_FORMAT_EVT21;
		genx320->format_code = MEDIA_BUS_FMT_PSEE_EVT21;
		break;
	case 2:
		core->format = EVENT_FORMAT_EVT3;
		genx320->format_code = MEDIA_BUS_FMT_PSEE_EVT3;
		break;
	default:
		dev_err(pcw->sd.dev, "Invalid format code\n");
		return -EINVAL;
	}

	RET_ON(genx320_apply_format(genx320));

	return 0;
}

static const struct v4l2_ctrl_ops evt_format_ctrl_ops = {
	.s_ctrl = evt_format_s_ctrl,
};

static const char * const evt_format_names[] = {
	"EVT2",
	"EVT21",
	"EVT3",
};

static const struct v4l2_ctrl_config evt_format_cfg = {
    .id            = PSEE_CID_EVT_FORMAT,
    .name          = "evt_format",
    .type          = V4L2_CTRL_TYPE_MENU,
    .min           = 0,
    .max           = ARRAY_SIZE(evt_format_names) - 1,
    .def           = 1, // Default to EVT21 which is set internally in probe
    .menu_skip_mask = 0,
    .qmenu          = evt_format_names,
	.ops 			= &evt_format_ctrl_ops,
};
#endif

/**
 * genx320_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_probe(struct i2c_client *client)
{
	struct genx320 *genx320;
	struct v4l2_ctrl *ctrl;
	const char *name;
	int ret;

	genx320 = devm_kzalloc(&client->dev, sizeof(*genx320), GFP_KERNEL);
	if (!genx320)
		return -ENOMEM;

	genx320->pcw.dev = &client->dev;
	name = device_get_match_data(&client->dev);
	if (!name)
		return -ENODEV;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&genx320->pcw.sd, client, &genx320_subdev_ops);
	genx320->pcw.sd.internal_ops = &genx320_internal_ops;
	
	ret = genx320_parse_hw_config(genx320);
	if (ret) {
		dev_err(genx320->pcw.dev, "HW configuration is not supported");
		return ret;
	}

	mutex_init(&genx320->mutex);

	/* Set default output format */
	genx320->format_code = supported_formats[1];
	genx320_init_controls(genx320, &genx320_ctrl_ops);

	/* Advertise the only supported link frequency */
	ctrl = v4l2_ctrl_new_int_menu(&genx320->pcw.hdl,
		NULL, V4L2_CID_LINK_FREQ, 0, 0, &link_freq[0]);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

#ifdef OMIT_PSEE_FORMATS
	struct v4l2_ctrl *ctrl_format;
	ctrl_format = v4l2_ctrl_new_custom(&genx320->pcw.hdl, &evt_format_cfg, NULL);
#endif

	ret = genx320_power_on(genx320->pcw.dev);
	if (ret) {
		dev_err(genx320->pcw.dev, "failed to power-on the sensor");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = genx320_detect(genx320);
	if (ret) {
		dev_err(genx320->pcw.dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	genx320->pcw.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	genx320->pcw.sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	v4l2_i2c_subdev_set_name(&genx320->pcw.sd, client, name, NULL);

	/* Initialize source pad */
	genx320->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&genx320->pcw.sd.entity, 1, &genx320->pad);
	if (ret) {
		dev_err(genx320->pcw.dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&genx320->pcw.sd);
	if (ret < 0) {
		dev_err(genx320->pcw.dev,
			"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(genx320->pcw.dev);
	pm_runtime_enable(genx320->pcw.dev);
	pm_runtime_idle(genx320->pcw.dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&genx320->pcw.sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(genx320->pcw.sd.ctrl_handler);
error_power_off:
	genx320_power_off(genx320->pcw.dev);
error_mutex_destroy:
	mutex_destroy(&genx320->mutex);

	return ret;
}

/**
 * genx320_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 */
static void genx320_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct genx320 *genx320 = to_genx320(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		genx320_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&genx320->mutex);
}

static const struct dev_pm_ops genx320_pm_ops = {
	SET_RUNTIME_PM_OPS(genx320_power_off, genx320_power_on, NULL)
};

static const struct of_device_id genx320_of_match[] = {
	{ .compatible = "psee,genx320", .data = "genx320" },
	{ }
};

MODULE_DEVICE_TABLE(of, genx320_of_match);

static struct i2c_driver genx320_driver = {
	.probe = genx320_probe,
	.remove = genx320_remove,
	.driver = {
		.name = "genx320",
		.pm = &genx320_pm_ops,
		.of_match_table = genx320_of_match,
	},
};

module_i2c_driver(genx320_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prophesee");
MODULE_DESCRIPTION("Prophesee genx320 sensor driver");
