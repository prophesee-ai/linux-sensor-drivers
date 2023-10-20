// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony imx636 Camera Sensor Driver
 *
 * Copyright (C) 2023 Prophesee
 */
#include <asm/unaligned.h>

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

/*
 * Sensor registers
 */

#define IMX636_CHIP_ID 0x14
#define IMX636_ID 0xA0401806

#define IMX636_STANDBY_CTRL 0xC8
#define IMX636_STANDBY_VALUE 0x101

/* MBX registers */
#define MBX_BASE 0x400000

#define IMX636_MBX_MISC (MBX_BASE + 0x010)
#define IMX636_BOOT_MAGIC 3405691582u

static const char * const imx636_supply_names[] = {
	"vadd",		/* Supply voltage (Analog) */
	"vddd1",	/* Supply voltage (Digital 1) */
	"vddd2",	/* Supply voltage (Digital 2) */
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
 * @link_freq: frequency of the CSI-2 clock lane
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
	s64 link_freq;
	u32 format_code;
	bool streaming;
};

static const s64 link_freq[] = {
	1500000000,
};

/* Supported sensor media formats */
static const u32 supported_formats[] = {
	MEDIA_BUS_FMT_PSEE_EVT3,
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

	mutex_lock(&imx636->mutex);

	code = supported_formats[0];
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
 * imx636_start_streaming() - Start sensor stream
 * @imx636: pointer to imx636 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx636_start_streaming(struct imx636 *imx636)
{
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
		for (j = 0; j < ARRAY_SIZE(link_freq); j++) {
			if (bus_cfg.link_frequencies[i] == link_freq[j]) {
				dev_info(imx636->dev, "Using CSI-2 freq %lld", link_freq[j]);
				imx636->link_freq = link_freq[j];
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

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx636_video_ops = {
	.s_stream = imx636_set_stream,
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
