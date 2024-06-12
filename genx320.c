// SPDX-License-Identifier: GPL-2.0-only
/*
 * Prophesee genx320 Camera Sensor Driver
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

#define GENX320_PIXEL_ARRAY_WIDTH 320U
#define GENX320_PIXEL_ARRAY_HEIGHT 320U

#define GENX320_NUM_DATA_LANES 1
#define GENX320_INCLK_RATE 20000000

/*
 * Sensor registers
 */

#define GENX320_CHIP_ID 0x14
#define GENX320_ID 0xb0602003

/* MBX registers */
#define MBX_BASE 0xF000

#define GENX320_MBX_MISC (MBX_BASE + 0x0010)
#define GENX320_BOOT_MAGIC 3405691582u

static const char * const genx320_supply_names[] = {
	"vadd",		/* Supply voltage (Analog) */
	"vddd1",	/* Supply voltage (Digital 1) */
	"vddd2",	/* Supply voltage (Digital 2) */
};

/**
 * struct genx320 - genx320 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @nreset_gpio: Sensor RSTn gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @mutex: Mutex for serializing sensor controls
 * @link_freq: frequency of the CSI-2 clock lane
 * @format_code: Media-ctl code of the output format
 * @streaming: Flag indicating streaming state
 */
struct genx320 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *nreset_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(genx320_supply_names)];
	struct mutex mutex;
	s64 link_freq;
	u32 format_code;
	bool streaming;
};

static const s64 link_freq[] = {
	600000000,
};

/* Supported sensor media formats */
static const u32 supported_formats[] = {
	MEDIA_BUS_FMT_PSEE_EVT3,
};


/**
 * to_genx320() - genx320 V4L2 sub-device to genx320 device.
 * @subdev: pointer to genx320 V4L2 sub-device
 *
 * Return: pointer to genx320 device
 */
static inline struct genx320 *to_genx320(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct genx320, sd);
}

/**
 * genx320_read_reg() - Read registers.
 * @genx320: pointer to genx320 device
 * @reg: register address
 * @len: length of registers
 * @val: pointer to register array to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_read_reg(struct genx320 *genx320, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&genx320->sd);
	struct i2c_msg xfer[2] = {0};
	int i, ret;

	xfer[0].addr = client->addr;
	reg = cpu_to_be16(reg);
	xfer[0].buf = (u8 *)&reg;
	xfer[0].len = sizeof(reg);
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = (u8 *)val;
	xfer[1].len = len * sizeof(*val);

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret != 2) {
		dev_warn(genx320->dev, "read ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	} else {
		for (i = 0; i < len; i++)
			val[i] = be32_to_cpu(val[i]);
		ret = 0;
	}

	return ret;
}

/**
 * genx320_write_reg() - Write one register
 * @genx320: pointer to genx320 device
 * @reg: register address
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_write_reg(struct genx320 *genx320, u16 reg, const u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&genx320->sd);
	struct i2c_msg xfer = {0};
	u8 buf[sizeof(reg) + sizeof(val)] = {0};
    u16 *regp = (u16 *)&buf[0];
    u32 *valp = (u32 *)&buf[2];
	int ret;

	xfer.addr = client->addr;
	*regp = cpu_to_be16(reg);
	*valp = cpu_to_be32(val);

	xfer.buf = buf;
	xfer.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret > 0) {
		ret = 0;
	} else {
		dev_warn(genx320->dev, "write ret %d", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	return ret;
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
	if (code->index >= ARRAY_SIZE(supported_formats))
		return -EINVAL;

	code->code = supported_formats[code->index];

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

	fsize->min_width = GENX320_PIXEL_ARRAY_WIDTH;
	fsize->max_width = fsize->min_width;
	fsize->min_height = GENX320_PIXEL_ARRAY_HEIGHT;
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
	fmt->format.width = GENX320_PIXEL_ARRAY_WIDTH;
	fmt->format.height = GENX320_PIXEL_ARRAY_HEIGHT;
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

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		genx320_fill_pad_format(genx320, genx320->format_code, fmt);
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

	code = supported_formats[0];
	genx320_fill_pad_format(genx320, code, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		genx320->format_code = code;
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
	genx320_fill_pad_format(genx320, supported_formats[0], &fmt);

	return genx320_set_pad_format(sd, sd_state, &fmt);
}

/**
 * genx320_start_streaming() - Start sensor stream
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_start_streaming(struct genx320 *genx320)
{
	return 0;
}

/**
 * genx320_stop_streaming() - Stop sensor stream
 * @genx320: pointer to genx320 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_stop_streaming(struct genx320 *genx320)
{
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

	if (genx320->streaming == enable) {
		mutex_unlock(&genx320->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(genx320->dev);
		if (ret)
			goto error_unlock;

		ret = genx320_start_streaming(genx320);
		if (ret)
			goto error_power_off;
	} else {
		genx320_stop_streaming(genx320);
		pm_runtime_put(genx320->dev);
	}

	genx320->streaming = enable;

	mutex_unlock(&genx320->mutex);

	return 0;

error_power_off:
	pm_runtime_put(genx320->dev);
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

	ret = genx320_read_reg(genx320, GENX320_CHIP_ID, 1, &val);
	if (ret)
		return ret;

	if (val != GENX320_ID) {
		dev_err(genx320->dev, "chip id mismatch: %x!=%x",
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
	struct fwnode_handle *fwnode = dev_fwnode(genx320->dev);
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
	genx320->nreset_gpio = devm_gpiod_get_optional(genx320->dev, "nreset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(genx320->nreset_gpio)) {
		dev_err(genx320->dev, "failed to get reset gpio %ld",
			PTR_ERR(genx320->nreset_gpio));
		return PTR_ERR(genx320->nreset_gpio);
	}

	/* Get sensor input clock */
	genx320->inclk = devm_clk_get(genx320->dev, NULL);
	if (IS_ERR(genx320->inclk)) {
		dev_err(genx320->dev, "could not get inclk");
		return PTR_ERR(genx320->inclk);
	}

	rate = clk_get_rate(genx320->inclk);
	if (rate != GENX320_INCLK_RATE) {
		dev_err(genx320->dev, "inclk frequency mismatch");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(genx320_supply_names); i++)
		genx320->supplies[i].supply = genx320_supply_names[i];

	ret = devm_regulator_bulk_get(genx320->dev,
				      ARRAY_SIZE(genx320_supply_names),
				      genx320->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GENX320_NUM_DATA_LANES) {
		dev_err(genx320->dev,
			"number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(genx320->dev, "no link frequencies defined");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(link_freq); j++) {
			if (bus_cfg.link_frequencies[i] == link_freq[j]) {
				dev_info(genx320->dev, "Using CSI-2 freq %lld", link_freq[j]);
				genx320->link_freq = link_freq[j];
				goto done_endpoint_free;
			}
		}
	}

	dev_err(genx320->dev, "none of the link frequencies is supported");
	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops genx320_video_ops = {
	.s_stream = genx320_set_stream,
};

static const struct v4l2_subdev_pad_ops genx320_pad_ops = {
	.init_cfg = genx320_init_pad_cfg,
	.enum_mbus_code = genx320_enum_mbus_code,
	.enum_frame_size = genx320_enum_frame_size,
	.get_fmt = genx320_get_pad_format,
	.set_fmt = genx320_set_pad_format,
};

static const struct v4l2_subdev_ops genx320_subdev_ops = {
	.video = &genx320_video_ops,
	.pad = &genx320_pad_ops,
};

/**
 * genx320_check_boot() - Check the boot magic
 * @genx320: pointer to the genx320 device
 *
 * There is a misc register switching to a magic value at the end of sensor boot process
 * It should be covered by the wait time given by Prophesee, thus this is just a sanity check
 */
static void genx320_check_boot(struct genx320 *genx320)
{
	int ret;
	u32 val;

	ret = genx320_read_reg(genx320, GENX320_MBX_MISC, 1, &val);
	if (ret)
		dev_warn(genx320->dev, "could not get the boot magic");
	if (val != GENX320_BOOT_MAGIC)
		dev_warn(genx320->dev, "unexpected boot magic, got %u, expected %u",
			val, GENX320_BOOT_MAGIC);
}

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
		dev_err(genx320->dev, "fail to enable inclk");
		goto error_reset;
	}

	/* Trstn_start = 200ns (min) */
	usleep_range(1, 100);
	gpiod_set_value_cansleep(genx320->nreset_gpio, 1);

	/* Tstart = 15ms (min) */
	/* but CCAM5 introduces 48ms +/-15% delay */
	msleep_interruptible(15 + 55);
	genx320_check_boot(genx320);

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

	return 0;
}

/**
 * genx320_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_probe(struct i2c_client *client)
{
	struct genx320 *genx320;
	const char *name;
	int ret;

	genx320 = devm_kzalloc(&client->dev, sizeof(*genx320), GFP_KERNEL);
	if (!genx320)
		return -ENOMEM;

	genx320->dev = &client->dev;
	name = device_get_match_data(&client->dev);
	if (!name)
		return -ENODEV;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&genx320->sd, client, &genx320_subdev_ops);

	ret = genx320_parse_hw_config(genx320);
	if (ret) {
		dev_err(genx320->dev, "HW configuration is not supported");
		return ret;
	}

	mutex_init(&genx320->mutex);

	ret = genx320_power_on(genx320->dev);
	if (ret) {
		dev_err(genx320->dev, "failed to power-on the sensor");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = genx320_detect(genx320);
	if (ret) {
		dev_err(genx320->dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	/* Set default output format */
	genx320->format_code = supported_formats[0];

	/* Initialize subdev */
	genx320->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	genx320->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	v4l2_i2c_subdev_set_name(&genx320->sd, client, name, NULL);

	/* Initialize source pad */
	genx320->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&genx320->sd.entity, 1, &genx320->pad);
	if (ret) {
		dev_err(genx320->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&genx320->sd);
	if (ret < 0) {
		dev_err(genx320->dev,
			"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(genx320->dev);
	pm_runtime_enable(genx320->dev);
	pm_runtime_idle(genx320->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&genx320->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(genx320->sd.ctrl_handler);
error_power_off:
	genx320_power_off(genx320->dev);
error_mutex_destroy:
	mutex_destroy(&genx320->mutex);

	return ret;
}

/**
 * genx320_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int genx320_remove(struct i2c_client *client)
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
	return 0;
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
	.probe_new = genx320_probe,
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
