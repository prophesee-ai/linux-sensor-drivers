// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony imx636 Camera Sensor Driver
 *
 * Copyright (C) 2023 Prophesee
 */
#include <asm/unaligned.h>

#include <linux/kconfig.h> /* to detect big-endian builds */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "psee-format.h"

#define IMX636_PIXEL_ARRAY_WIDTH 1280U
#define IMX636_PIXEL_ARRAY_HEIGHT 720U

#define IMX636_NUM_DATA_LANES 2
#define IMX636_INCLK_RATE 20000000

/* to avoid return value check on each register access */
#define RET_ON(operation) do { int r = (operation); if (unlikely(r != 0)) return r; } while (0)

/*
 * Sensor registers
 */
#define IMX636_GLOBAL_CTRL 0x00
#define IMX636_SYS_CLK_SWITCH_SEL BIT(3)
#define IMX636_SYS_CLK_EN BIT(30)

#define IMX636_ROI_CTRL 0x04
#define IMX636_ROI_PX_TD_RSTN BIT(10)

#define IMX636_CHIP_ID 0x14
#define IMX636_ID 0xA0401806

#define IMX636_DV_CTRL 0xB8
#define IMX636_DV_PC_CLKDIVEN BIT(0)
#define IMX636_DV_PC_SYSCLKEN BIT(3)

#define IMX636_GLOBAL_CTRL2 0xC0
#define IMX636_DVTOP_DIVIDER_MASK (0xFF)
#define IMX636_DVTOP_DIVIDER(div) ((div) & IMX636_DVTOP_DIVIDER_MASK)
#define IMX636_SYS_CLK_DIVIDER_NEW_MASK (0x700)
#define IMX636_SYS_CLK_DIVIDER_NEW(div) (((div) << 8) & IMX636_SYS_CLK_DIVIDER_NEW_MASK)

#define IMX636_STANDBY_CTRL 0xC8
#define IMX636_STANDBY_VALUE 0x101

/* EDF registers */
#define EDF_BASE 0x7000

#define IMX636_EDF_PIPELINE_CONTROL (EDF_BASE + 0x000)
#define IMX636_EDF_PIPELINE_EVT3 (0x00070001)
#define IMX636_EDF_PIPELINE_EVT21 (0x00070003)

/* EOI registers */
#define EOI_BASE 0x8000

#define IMX636_EOI_PIPELINE_CONTROL (EOI_BASE + 0x000)
#define IMX636_EOI_BYTE_ORDER_MASK 0xC0
#define IMX636_EOI_BYTE_ORDER_32LE 0x00
#define IMX636_EOI_BYTE_ORDER_16LE 0x80
#define IMX636_EOI_BYTE_ORDER_32BE 0xC0

/* RO registers */
#define RO_BASE 0x9000

#define IMX636_RO_TIME_BASE_CTRL (RO_BASE + 0x008)
#define IMX636_RO_TIME_BASE_ENABLE BIT(0)

#define IMX636_RO_LP_CTRL (RO_BASE + 0x028)
#define IMX636_LP_OUTPUT_DISABLE BIT(1)

/* MIPI_CSI registers */
#define MIPI_CSI_BASE 0xB000

#define IMX636_MIPI_CONTROL (MIPI_CSI_BASE + 0x000)
#define IMX636_MIPI_CSI_ENABLE BIT(0)

#define IMX636_MIPI_ESCAPE_CTRL (MIPI_CSI_BASE + 0x004)
#define IMX636_MIPI_ESCAPE_CLK_EN BIT(7)

#define IMX636_MIPI_PL_RG_1 (MIPI_CSI_BASE + 0x064)
#define IMX636_MIPI_PL_RG_CKOUTEN BIT(1)

#define IMX636_MIPI_PL_RG_2 (MIPI_CSI_BASE + 0x068)
#define IMX636_MIPI_PL_RG_ENDET_OP BIT(2)

#define IMX636_MIPI_PL_RG_5 (MIPI_CSI_BASE + 0x074)

#define IMX636_MIPI_PL_RG_6 (MIPI_CSI_BASE + 0x078)

#define IMX636_MIPI_PL_RG_7 (MIPI_CSI_BASE + 0x07C)
#define IMX636_MIPI_PL_XCLR_LV BIT(0)
#define IMX636_MIPI_PL_XSTB_OP BIT(1)
#define IMX636_MIPI_PL_CLK_LOCKDET_OP BIT(2)

#define IMX636_MIPI_POWER (MIPI_CSI_BASE + 0x040)
#define IMX636_MIPI_POWER_RST_W BIT(0) /* async nrst */
#define IMX636_MIPI_POWER_XCLR_LV BIT(1)
#define IMX636_MIPI_POWER_XCLR_MV BIT(2)
#define IMX636_MIPI_POWER_BCIF_EN BIT(3)

#define IMX636_MIPI_STREAM (MIPI_CSI_BASE + 0x044)

#define IMX636_MIPI_TCLKPOST (MIPI_CSI_BASE + 0x080)
#define IMX636_MIPI_TCLKPRE (MIPI_CSI_BASE + 0x084)
#define IMX636_MIPI_TCLKPREPARE (MIPI_CSI_BASE + 0x088)
#define IMX636_MIPI_TCLKTRAIL (MIPI_CSI_BASE + 0x08C)
#define IMX636_MIPI_TCLKZERO (MIPI_CSI_BASE + 0x090)
#define IMX636_MIPI_THSEXIT (MIPI_CSI_BASE + 0x094)
#define IMX636_MIPI_THSPREPARE (MIPI_CSI_BASE + 0x098)
#define IMX636_MIPI_THSZERO (MIPI_CSI_BASE + 0x09C)
#define IMX636_MIPI_THSTRAIL (MIPI_CSI_BASE + 0x0A0)
#define IMX636_MIPI_TLPX (MIPI_CSI_BASE + 0x0A4)
#define IMX636_MIPI_TXCLKESC_FREQ (MIPI_CSI_BASE + 0x0AC)

#define IMX636_MIPI_DPHY_POWER (MIPI_CSI_BASE + 0x0C8)
#define IMX636_MIPI_RG_BIASEN BIT(0)
#define IMX636_MIPI_RG_LPREGEN BIT(1)

#define IMX636_MIPI_DPHY_PLL_DIV (MIPI_CSI_BASE + 0x0CC)

#define IMX636_MIPI_BYTECLK_CTRL (MIPI_CSI_BASE + 0x120)

/* SLVS registers */
#define MIPI_SLVS_BASE 0xE000

#define IMX636_MIPI_SLVS_LINKCLK_CTRL (MIPI_SLVS_BASE + 0x120)

/* MBX registers */
#define MBX_BASE 0x400000

#define IMX636_MBX_MISC (MBX_BASE + 0x010)
#define IMX636_BOOT_MAGIC 3405691582u

static const char * const imx636_supply_names[] = {
	"vadd",		/* Supply voltage (Analog) */
	"vddd1",	/* Supply voltage (Digital 1) */
	"vddd2",	/* Supply voltage (Digital 2) */
};

/* For now, the 4 available configs are pre-computed */
static const struct link_timing {
	s64 line_freq;
	u32 pll_fb_div_d;
	u32 dvtop_div_d;
	u32 sys_clk_div_d;
	u16 tclkpost;
	u16 tclkpre;
	u16 tclkprepare;
	u16 tclktrail;
	u16 tclkzero;
	u16 thsexit;
	u16 thsprepare;
	u16 thszero;
	u16 thstrail;
	u16 tlpx;
	u16 txclkesc_freq;
	u8 dphy_clk_div;
} link_timings[] = {
	{ .line_freq = 1500000000, .dphy_clk_div = 0,
	 .pll_fb_div_d = 150, .dvtop_div_d = 0x20, .sys_clk_div_d = 1,
	 .tclkpost = 167, .tclkpre = 15, .tclkprepare = 87, .tclktrail = 95, .tclkzero = 407,
	 .thsexit = 159, .thsprepare = 95, .thszero = 175, .thstrail = 95, .tlpx = 79,
	 .txclkesc_freq = 40,
	},
	{ .line_freq = 1200000000, .dphy_clk_div = 0,
	 .pll_fb_div_d = 120, .dvtop_div_d = 0x08, .sys_clk_div_d = 2,
	 .tclkpost = 151, .tclkpre = 15, .tclkprepare = 79, .tclktrail = 79, .tclkzero = 335,
	 .thsexit = 135, .thsprepare = 79, .thszero = 151, .thstrail = 79, .tlpx = 63,
	 .txclkesc_freq = 40,
	},
	{ .line_freq = 800000000, .dphy_clk_div = 1,
	 .pll_fb_div_d = 160, .dvtop_div_d = 0x10, .sys_clk_div_d = 2,
	 .tclkpost = 119, .tclkpre = 15, .tclkprepare = 55, .tclktrail = 55, .tclkzero = 223,
	 .thsexit = 87, .thsprepare = 55, .thszero = 103, .thstrail = 55, .tlpx = 47,
	 .txclkesc_freq = 40,
	},
	{ .line_freq = 600000000, .dphy_clk_div = 1,
	 .pll_fb_div_d = 120, .dvtop_div_d = 0x08, .sys_clk_div_d = 2,
	 .tclkpost = 103, .tclkpre = 15, .tclkprepare = 39, .tclktrail = 39, .tclkzero = 183,
	 .thsexit = 71, .thsprepare = 47, .thszero = 79, .thstrail = 47, .tlpx = 39,
	 .txclkesc_freq = 40,
	},
};

/**
 * struct imx636 - imx636 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @nreset_gpio: Sensor RSTn gpio
 * @xclr_gpio: Sensor XCLR gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @mutex: Mutex for serializing sensor controls
 * @link_timing: Pointer to pre-computed timing for the CSI-2 link
 * @format_code: Media-ctl code of the output format
 * @streaming: Flag indicating streaming state
 */
struct imx636 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *nreset_gpio;
	struct gpio_desc *xclr_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx636_supply_names)];
	struct mutex mutex;
	const struct link_timing *timings;
	u32 format_code;
	bool streaming;
};

/* Supported sensor media formats */
static const u32 supported_formats[] = {
	MEDIA_BUS_FMT_PSEE_EVT3,
	MEDIA_BUS_FMT_PSEE_EVT21,
};


/**
 * to_imx636() - imx636 V4L2 sub-device to imx636 device.
 * @subdev: pointer to imx636 V4L2 sub-device
 *
 * Return: pointer to imx636 device
 */
static inline struct imx636 *to_imx636(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx636, sd);
}

/**
 * imx636_read_reg() - Read registers.
 * @imx636: pointer to imx636 device
 * @reg: register address
 * @len: length of registers
 * @val: pointer to register array to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_read_reg(struct imx636 *imx636, u32 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx636->sd);
	struct i2c_msg xfer[2] = {0};
	int i, ret;

	xfer[0].addr = client->addr;
	reg = cpu_to_be32(reg);
	xfer[0].buf = (u8 *)&reg;
	xfer[0].len = sizeof(reg);
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = (u8 *)val;
	xfer[1].len = len * sizeof(*val);

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret != 2) {
		dev_warn(imx636->dev, "read ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	} else {
		for (i = 0; i < len; i++)
			val[i] = be32_to_cpu(val[i]);
		ret = 0;
	}

	return ret;
}

/**
 * imx636_write_reg() - Write one register
 * @imx636: pointer to imx636 device
 * @reg: register address
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_write_reg(struct imx636 *imx636, u32 reg, const u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx636->sd);
	struct i2c_msg xfer = {0};
	u32 buf[2] = {0};
	int ret;

	xfer.addr = client->addr;
	buf[0] = cpu_to_be32(reg);
	buf[1] = cpu_to_be32(val);
	xfer.buf = (u8 *)buf;
	xfer.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret > 0) {
		ret = 0;
	} else {
		dev_warn(imx636->dev, "write ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	return ret;
}

/**
 * imx636_set_bitfield() - Set bits in a register to a given value
 * @imx636: pointer to imx636 device
 * @reg: register address
 * @field: bits to set
 * @val: value of the bits to fill, already shifted to match register layout
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_set_bitfield(struct imx636 *imx636, const u32 reg, const u32 field, const u32 val)
{
	u32 value, ret;

	ret = imx636_read_reg(imx636, reg, 1, &value);
	if (ret)
		return ret;
	value &= ~field;
	value |= val & field;
	return imx636_write_reg(imx636, reg, value);
}

/**
 * imx636_set_reg() - Set bits in a register
 * @imx636: pointer to imx636 device
 * @reg: register address
 * @bits: bits to set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_set_reg(struct imx636 *imx636, u32 reg, const u32 bits)
{
	return imx636_set_bitfield(imx636, reg, bits, ~0);
}

/**
 * imx636_write_reg() - Clear bits in a register
 * @imx636: pointer to imx636 device
 * @reg: register address
 * @bits: bits to clear
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_clear_reg(struct imx636 *imx636, u32 reg, const u32 bits)
{
	return imx636_set_bitfield(imx636, reg, bits, 0);
}

/**
 * imx636_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx636 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_formats))
		return -EINVAL;

	code->code = supported_formats[code->index];

	return 0;
}

/**
 * imx636_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx636 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index != 0)
		return -EINVAL;

	fsize->min_width = IMX636_PIXEL_ARRAY_WIDTH;
	fsize->max_width = fsize->min_width;
	fsize->min_height = IMX636_PIXEL_ARRAY_HEIGHT;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * imx636_fill_pad_format() - Fill subdevice pad format
 *                            from selected media format
 * @imx636: pointer to imx636 device
 * @code: media-ctl format code in use
 * @fmt: V4L2 sub-device format need to be filled
 */
static void imx636_fill_pad_format(struct imx636 *imx636,
				   u32 code,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX636_PIXEL_ARRAY_WIDTH;
	fmt->format.height = IMX636_PIXEL_ARRAY_HEIGHT;
	fmt->format.code = code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * imx636_get_pad_format() - Get subdevice pad format
 * @sd: pointer to imx636 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx636 *imx636 = to_imx636(sd);

	mutex_lock(&imx636->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx636_fill_pad_format(imx636, imx636->format_code, fmt);
	}

	mutex_unlock(&imx636->mutex);

	return 0;
}

/**
 * imx636_set_pad_format() - Set subdevice pad format
 * @sd: pointer to imx636 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx636 *imx636 = to_imx636(sd);
	u32 code;
	int ret = 0;
	int i;

	mutex_lock(&imx636->mutex);

	code = supported_formats[0];
	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (supported_formats[i] == fmt->format.code)
			code = supported_formats[i];
	}
	imx636_fill_pad_format(imx636, code, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx636->format_code = code;
	}

	mutex_unlock(&imx636->mutex);

	return ret;
}

/**
 * imx636_init_pad_cfg() - Initialize sub-device pad configuration
 * @sd: pointer to imx636 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_init_pad_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state)
{
	struct imx636 *imx636 = to_imx636(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	imx636_fill_pad_format(imx636, supported_formats[0], &fmt);

	return imx636_set_pad_format(sd, sd_state, &fmt);
}

/**
 * imx636_apply_format() - Set the sensor to output the selected format
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_apply_format(struct imx636 *imx636)
{
	int ret;
	u32 eoi_value;
	u32 byte_order;

	/* From CSI-2 point of view, the data is always "User Defined 8-bit Data Type 1",
	 * (cf Section 11.5 of CSI-2 Specification).
	 * To ease the decoding, the sensor reorders multi-byte data to match receiver
	 * byte-ordering.
	 */
	ret = imx636_read_reg(imx636, IMX636_EOI_PIPELINE_CONTROL, 1, &eoi_value);
	if (ret)
		return ret;

	switch (imx636->format_code) {
	case MEDIA_BUS_FMT_PSEE_EVT21:
		byte_order = IMX636_EOI_BYTE_ORDER_32LE;
		ret = imx636_write_reg(imx636, IMX636_EDF_PIPELINE_CONTROL,
			IMX636_EDF_PIPELINE_EVT21);
		if (ret)
			return ret;
		break;
	case MEDIA_BUS_FMT_PSEE_EVT3:
		byte_order = IMX636_EOI_BYTE_ORDER_16LE;
		ret = imx636_write_reg(imx636, IMX636_EDF_PIPELINE_CONTROL,
			IMX636_EDF_PIPELINE_EVT3);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	eoi_value &= ~IMX636_EOI_BYTE_ORDER_MASK;
#ifdef __BIG_ENDIAN
	/* On Big-endian architectures, no reordering should be necessary */
	byte_order = IMX636_EOI_BYTE_ORDER_32BE;
#endif
	eoi_value |= byte_order;
	return imx636_write_reg(imx636, IMX636_EOI_PIPELINE_CONTROL, eoi_value);
}

/**
 * imx636_reconfigure_csi2_freq() - Reconfigure the clock tree for the selected CSI-2 freq
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_reconfigure_csi2_freq(struct imx636 *imx636)
{
	/* The sensor starts with lanes at 1.5Gbps, which provides top performances, but some
	 * hardware may require to lower this frequency to preserve data integrity.
	 */
	int i;

	/* Disable MIPI CSI-2 */
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_CONTROL, IMX636_MIPI_CSI_ENABLE));

	/* Power down CSI-2 and D-PHY */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_STREAM, 0));
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_ESCAPE_CTRL, IMX636_MIPI_ESCAPE_CLK_EN));
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_POWER, IMX636_MIPI_POWER_RST_W));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_DPHY_POWER, 0));
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_POWER, IMX636_MIPI_POWER_XCLR_LV));
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_POWER, IMX636_MIPI_POWER_BCIF_EN));

	/* Power down PLL */
	RET_ON(imx636_clear_reg(imx636, IMX636_GLOBAL_CTRL, IMX636_SYS_CLK_SWITCH_SEL));
	RET_ON(imx636_clear_reg(imx636, IMX636_GLOBAL_CTRL, IMX636_SYS_CLK_EN));
	RET_ON(imx636_clear_reg(imx636, IMX636_DV_CTRL, IMX636_DV_PC_SYSCLKEN));
	RET_ON(imx636_clear_reg(imx636, IMX636_DV_CTRL, IMX636_DV_PC_CLKDIVEN));
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_PL_RG_7,
		IMX636_MIPI_PL_XCLR_LV | IMX636_MIPI_PL_XSTB_OP));

	/* reconfigure the PLL */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_PL_RG_5, 2 /* Input divider */));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_PL_RG_6, imx636->timings->pll_fb_div_d));
	RET_ON(imx636_write_reg(imx636, IMX636_GLOBAL_CTRL2,
		IMX636_DVTOP_DIVIDER(imx636->timings->dvtop_div_d) |
		IMX636_SYS_CLK_DIVIDER_NEW(imx636->timings->sys_clk_div_d)));

	/* Power up PLL */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_BYTECLK_CTRL, 1));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_SLVS_LINKCLK_CTRL, 0));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_PL_RG_2, IMX636_MIPI_PL_RG_ENDET_OP));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_PL_RG_7, IMX636_MIPI_PL_XCLR_LV));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_PL_RG_7, IMX636_MIPI_PL_XSTB_OP));

	/* At 1MHz, a register read takes more that 50us, the PLL should lock in 200us */
	for (i = 0; i < 10; i++) {
		u32 reg;

		RET_ON(imx636_read_reg(imx636, IMX636_MIPI_PL_RG_7, 1, &reg));
		if (reg & IMX636_MIPI_PL_CLK_LOCKDET_OP)
			break;
	}
	if (i == 10) {
		dev_err(imx636->dev, "Could not lock PLL");
		return -EIO;
	}
	RET_ON(imx636_set_reg(imx636, IMX636_DV_CTRL, IMX636_DV_PC_CLKDIVEN));
	RET_ON(imx636_set_reg(imx636, IMX636_DV_CTRL, IMX636_DV_PC_SYSCLKEN));
	RET_ON(imx636_set_reg(imx636, IMX636_GLOBAL_CTRL, IMX636_SYS_CLK_EN));
	RET_ON(imx636_set_reg(imx636, IMX636_GLOBAL_CTRL, IMX636_SYS_CLK_SWITCH_SEL));

	/* Update the D-PHY timings for the new clock configuration */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TCLKPOST, imx636->timings->tclkpost));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TCLKPRE, imx636->timings->tclkpre));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TCLKPREPARE, imx636->timings->tclkprepare));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TCLKTRAIL, imx636->timings->tclktrail));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TCLKZERO, imx636->timings->tclkzero));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_THSEXIT, imx636->timings->thsexit));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_THSPREPARE, imx636->timings->thsprepare));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_THSZERO, imx636->timings->thszero));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_THSTRAIL, imx636->timings->thstrail));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TLPX, imx636->timings->tlpx));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_TXCLKESC_FREQ, imx636->timings->txclkesc_freq));
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_DPHY_PLL_DIV, imx636->timings->dphy_clk_div));

	/* Power up D-PHY */
	/* The LDOs are still up, no need to re-enable them */
	/* Re-enable power, except reference bias current */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_POWER,
		IMX636_MIPI_POWER_RST_W | IMX636_MIPI_POWER_XCLR_LV | IMX636_MIPI_POWER_XCLR_MV));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_PL_RG_1, IMX636_MIPI_PL_RG_CKOUTEN));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_POWER, IMX636_MIPI_POWER_BCIF_EN));
	usleep_range(100, 200);
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_ESCAPE_CTRL, IMX636_MIPI_ESCAPE_CLK_EN));
	usleep_range(200, 300);
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_DPHY_POWER,
		IMX636_MIPI_RG_BIASEN | IMX636_MIPI_RG_LPREGEN));
	usleep_range(200, 300);

	/* Re-enable stream and control */
	RET_ON(imx636_write_reg(imx636, IMX636_MIPI_STREAM, 1));
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_CONTROL, IMX636_MIPI_CSI_ENABLE));

	return 0;
}

/**
 * imx636_start_streaming() - Start sensor stream
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_start_streaming(struct imx636 *imx636)
{
	/* MIPI CSI-2 enable */
	RET_ON(imx636_set_reg(imx636, IMX636_MIPI_CONTROL, IMX636_MIPI_CSI_ENABLE));
	/* Pixel reset release */
	RET_ON(imx636_set_reg(imx636, IMX636_ROI_CTRL, IMX636_ROI_PX_TD_RSTN));
	/* Timer base enable */
	RET_ON(imx636_set_reg(imx636, IMX636_RO_TIME_BASE_CTRL, IMX636_RO_TIME_BASE_ENABLE));
	/* Digital data enable (only needed when resuming after suspend) */
	RET_ON(imx636_clear_reg(imx636, IMX636_RO_LP_CTRL, IMX636_LP_OUTPUT_DISABLE));
	return 0;
}

/**
 * imx636_stop_streaming() - Stop sensor stream
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_stop_streaming(struct imx636 *imx636)
{
	/* Skipping other accesses if one fail may not be the best policy, but overall,
	 * if we can't do register accesses, we're doomed, one way or another
	 */
	/* Digital data disable */
	RET_ON(imx636_set_reg(imx636, IMX636_RO_LP_CTRL, IMX636_LP_OUTPUT_DISABLE));
	/* Timer base disable */
	RET_ON(imx636_clear_reg(imx636, IMX636_RO_TIME_BASE_CTRL, IMX636_RO_TIME_BASE_ENABLE));
	/* Pixel reset release */
	RET_ON(imx636_clear_reg(imx636, IMX636_ROI_CTRL, IMX636_ROI_PX_TD_RSTN));
	/* MIPI CSI-2 disable */
	RET_ON(imx636_clear_reg(imx636, IMX636_MIPI_CONTROL, IMX636_MIPI_CSI_ENABLE));
	return 0;
}

/**
 * imx636_set_stream() - Enable sensor streaming
 * @sd: pointer to imx636 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx636 *imx636 = to_imx636(sd);
	int ret;

	mutex_lock(&imx636->mutex);

	if (imx636->streaming == enable) {
		mutex_unlock(&imx636->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(imx636->dev);
		if (ret)
			goto error_unlock;

		ret = imx636_reconfigure_csi2_freq(imx636);
		if (ret)
			goto error_power_off;

		ret = imx636_apply_format(imx636);
		if (ret)
			goto error_power_off;

		ret = imx636_start_streaming(imx636);
		if (ret)
			goto error_power_off;
	} else {
		imx636_stop_streaming(imx636);
		pm_runtime_put(imx636->dev);
	}

	imx636->streaming = enable;

	mutex_unlock(&imx636->mutex);

	return 0;

error_power_off:
	pm_runtime_put(imx636->dev);
error_unlock:
	mutex_unlock(&imx636->mutex);

	return ret;
}

/**
 * imx636_detect() - Detect imx636 sensor
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx636_detect(struct imx636 *imx636)
{
	int ret;
	u32 val;

	ret = imx636_read_reg(imx636, IMX636_CHIP_ID, 1, &val);
	if (ret)
		return ret;

	if (val != IMX636_ID) {
		dev_err(imx636->dev, "chip id mismatch: %x!=%x",
			IMX636_ID, val);
		return -ENXIO;
	}

	return 0;
}

/**
 * imx636_parse_hw_config() - Parse HW configuration and check if supported
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_parse_hw_config(struct imx636 *imx636)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx636->dev);
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
	imx636->nreset_gpio = devm_gpiod_get_optional(imx636->dev, "nreset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx636->nreset_gpio)) {
		dev_err(imx636->dev, "failed to get reset gpio %ld",
			PTR_ERR(imx636->nreset_gpio));
		return PTR_ERR(imx636->nreset_gpio);
	}

	/* Request optional xclr pin */
	imx636->xclr_gpio = devm_gpiod_get_optional(imx636->dev, "xclr",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx636->xclr_gpio)) {
		dev_err(imx636->dev, "failed to get xclr gpio %ld",
			PTR_ERR(imx636->xclr_gpio));
		return PTR_ERR(imx636->xclr_gpio);
	}

	/* Get sensor input clock */
	imx636->inclk = devm_clk_get(imx636->dev, NULL);
	if (IS_ERR(imx636->inclk)) {
		dev_err(imx636->dev, "could not get inclk");
		return PTR_ERR(imx636->inclk);
	}

	rate = clk_get_rate(imx636->inclk);
	if (rate != IMX636_INCLK_RATE) {
		dev_err(imx636->dev, "inclk frequency mismatch");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(imx636_supply_names); i++)
		imx636->supplies[i].supply = imx636_supply_names[i];

	ret = devm_regulator_bulk_get(imx636->dev,
				      ARRAY_SIZE(imx636_supply_names),
				      imx636->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX636_NUM_DATA_LANES) {
		dev_err(imx636->dev,
			"number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx636->dev, "no link frequencies defined");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(link_timings); j++) {
			if (bus_cfg.link_frequencies[i] == link_timings[j].line_freq) {
				imx636->timings = &link_timings[j];
				dev_info(imx636->dev, "Using CSI-2 freq %lld",
					imx636->timings->line_freq);
				goto done_endpoint_free;
			}
		}
	}

	dev_err(imx636->dev, "none of the link frequencies is supported");
	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int imx636_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct imx636 *imx636 = to_imx636(sd);
	u32 val;
	int ret;

	if (reg->size && reg->size != 4)
		return -EINVAL;

	ret = imx636_read_reg(imx636, (u32)reg->reg, 1, &val);
	reg->val = val;
	return ret;
}

static int imx636_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct imx636 *imx636 = to_imx636(sd);

	if (reg->size && reg->size != 4)
		return -EINVAL;

	return imx636_write_reg(imx636, (u32)reg->reg, (u32)reg->val);
}
#endif /* def CONFIG_VIDEO_ADV_DEBUG */

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx636_video_ops = {
	.s_stream = imx636_set_stream,
};

static const struct v4l2_subdev_core_ops imx636_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = imx636_g_register,
	.s_register = imx636_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops imx636_pad_ops = {
	.init_cfg = imx636_init_pad_cfg,
	.enum_mbus_code = imx636_enum_mbus_code,
	.enum_frame_size = imx636_enum_frame_size,
	.get_fmt = imx636_get_pad_format,
	.set_fmt = imx636_set_pad_format,
};

static const struct v4l2_subdev_ops imx636_subdev_ops = {
	.video = &imx636_video_ops,
	.pad = &imx636_pad_ops,
	.core = &imx636_core_ops,
};

/**
 * imx636_check_boot() - Check the boot magic
 * @imx636: pointer to the imx636 device
 *
 * There is a misc register switching to a magic value at the end of sensor boot process
 * It should be covered by the wait time given by Sony, thus this is just a sanity check
 */
static void imx636_check_boot(struct imx636 *imx636)
{
	int ret;
	u32 val;

	ret = imx636_read_reg(imx636, IMX636_MBX_MISC, 1, &val);
	if (ret)
		dev_warn(imx636->dev, "could not get the boot magic");
	if (val != IMX636_BOOT_MAGIC)
		dev_warn(imx636->dev, "unexpected boot magic, got %u, expected %u",
			val, IMX636_BOOT_MAGIC);
}

/**
 * imx636_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx636 *imx636 = to_imx636(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx636_supply_names),
				    imx636->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators");
		return ret;
	}


	gpiod_set_value_cansleep(imx636->xclr_gpio, 1);
	/* Txclr_start = 200ns (min) */
	usleep_range(1, 100);

	ret = clk_prepare_enable(imx636->inclk);
	if (ret) {
		dev_err(imx636->dev, "fail to enable inclk");
		goto error_reset;
	}

	/* Trstn_start = 200ns (min) */
	usleep_range(1, 100);
	gpiod_set_value_cansleep(imx636->nreset_gpio, 1);

	/* Tstart = 15ms (min) */
	/* but CCAM5 introduces 48ms +/-15% delay */
	msleep_interruptible(15 + 55);
	imx636_check_boot(imx636);

	return 0;

error_reset:
	gpiod_set_value_cansleep(imx636->nreset_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(imx636_supply_names),
			       imx636->supplies);

	return ret;
}

/**
 * imx636_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx636 *imx636 = to_imx636(sd);

	imx636_write_reg(imx636, IMX636_STANDBY_CTRL, IMX636_STANDBY_VALUE);
	/* Tstopwait = 15ms (min) */
	msleep_interruptible(15);

	gpiod_set_value_cansleep(imx636->nreset_gpio, 0);
	/* Tclk_pd = 200ns (min) */
	usleep_range(1, 100);

	clk_disable_unprepare(imx636->inclk);

	/* Txclr_pd = 200ns (min) */
	usleep_range(1, 100);
	gpiod_set_value_cansleep(imx636->xclr_gpio, 0);

	regulator_bulk_disable(ARRAY_SIZE(imx636_supply_names),
			       imx636->supplies);

	return 0;
}

/**
 * imx636_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_probe(struct i2c_client *client)
{
	struct imx636 *imx636;
	const char *name;
	int ret;

	imx636 = devm_kzalloc(&client->dev, sizeof(*imx636), GFP_KERNEL);
	if (!imx636)
		return -ENOMEM;

	imx636->dev = &client->dev;
	name = device_get_match_data(&client->dev);
	if (!name)
		return -ENODEV;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx636->sd, client, &imx636_subdev_ops);

	ret = imx636_parse_hw_config(imx636);
	if (ret) {
		dev_err(imx636->dev, "HW configuration is not supported");
		return ret;
	}

	mutex_init(&imx636->mutex);

	ret = imx636_power_on(imx636->dev);
	if (ret) {
		dev_err(imx636->dev, "failed to power-on the sensor");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = imx636_detect(imx636);
	if (ret) {
		dev_err(imx636->dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	/* Set default output format */
	imx636->format_code = supported_formats[0];

	/* Initialize subdev */
	imx636->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx636->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	v4l2_i2c_subdev_set_name(&imx636->sd, client, name, NULL);

	/* Initialize source pad */
	imx636->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx636->sd.entity, 1, &imx636->pad);
	if (ret) {
		dev_err(imx636->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx636->sd);
	if (ret < 0) {
		dev_err(imx636->dev,
			"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(imx636->dev);
	pm_runtime_enable(imx636->dev);
	pm_runtime_idle(imx636->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx636->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx636->sd.ctrl_handler);
error_power_off:
	imx636_power_off(imx636->dev);
error_mutex_destroy:
	mutex_destroy(&imx636->mutex);

	return ret;
}

/**
 * imx636_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx636 *imx636 = to_imx636(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx636_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx636->mutex);
	return 0;
}

static const struct dev_pm_ops imx636_pm_ops = {
	SET_RUNTIME_PM_OPS(imx636_power_off, imx636_power_on, NULL)
};

static const struct of_device_id imx636_of_match[] = {
	{ .compatible = "sony,imx636", .data = "imx636" },
	{ }
};

MODULE_DEVICE_TABLE(of, imx636_of_match);

static struct i2c_driver imx636_driver = {
	.probe_new = imx636_probe,
	.remove = imx636_remove,
	.driver = {
		.name = "imx636",
		.pm = &imx636_pm_ops,
		.of_match_table = imx636_of_match,
	},
};

module_i2c_driver(imx636_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prophesee");
MODULE_DESCRIPTION("Sony imx636 sensor driver");
