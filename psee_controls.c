// SPDX-License-Identifier: GPL-2.0-only

#include <media/v4l2-ctrls.h>
#include <linux/types.h>
#include "psee_controls.h"

static int s_roi_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);
	struct roi_set *roi_set;

	switch (ctrl->id) {
	case PSEE_CID_ROI_RESET:
		roi_set = pcw->roi_window_set->p_cur.p;
		roi_set->n = 0;
		return pcw->initialized ? call_esp_op(pcw, roi_window, reset): 0;
	case PSEE_CID_ROI_RONI:
		return call_esp_op(pcw, roi_window, enable_roni, ctrl->val);
	case PSEE_CID_ROI_WINDOW_SET:
		roi_set = ctrl->p_new.p;
		return call_esp_op(pcw, roi_window, set, roi_set->rois,  roi_set->n);
	case GENX320_CID_ROI_PIXEL_RESET:
		return call_esp_op(pcw, roi_pixel, reset);
	case GENX320_CID_ROI_PIXEL_ARRAY:
		struct grid *grid = ctrl->p_new.p;

		return call_esp_op(pcw, roi_pixel, set_array, grid);
	}

	return 0;
}

static int s_erc_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);
	struct v4l2_ctrl *rate = ctrl->cluster[1];

	if (ctrl->id != PSEE_CID_ERC_ENABLE)
		return -EINVAL;

	if (ctrl->val && *rate->p_new.p_s64 != *rate->p_cur.p_s64)
		RET_ON(call_esp_op(pcw, erc, s_rate, *rate->p_new.p_s64));

	return call_esp_op(pcw, erc, enable, ctrl->val);
}

static int s_bias_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);

	if (!has_esp_mod(pcw, bias))
		return -EINVAL;

	return call_esp_op(pcw, bias, set, ctrl->id - PSEE_CID_BIAS_CTRL, ctrl->val);
}

static int s_esp_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);

	// Button type controls may actually need to update the context
	if (!pcw->initialized && ctrl->type != V4L2_CTRL_TYPE_BUTTON)
		return 0;

	switch (PSEE_CTRL_CLASS(ctrl->id)) {
	case PSEE_ROI_CLASS:
		return s_roi_ctrl(ctrl);
	case PSEE_ERC_CLASS:
		return s_erc_ctrl(ctrl);
	case PSEE_BIAS_CLASS:
		return s_bias_ctrl(ctrl);
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops esp_ctrl_ops = {
	.s_ctrl = s_esp_ctrl,
};

static int stream_src_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);
	struct core_config *core = &pcw->controls.core;

	if (pcw->streaming) {
		dev_err(pcw->sd.dev, "Cannot change source while streaming\n");
		return -EBUSY;
	}

	switch (ctrl->val) {
	case PIXEL_ARRAY:
		core->source = SENSOR_SOURCE_PIXEL_ARRAY;
		break;
	case TIMEBASE_ONLY:
		core->source = SENSOR_SOURCE_TS_PATTERN;
		break;
	case RO_PATTERN:
		core->source = SENSOR_SOURCE_RO_PATTERN;
		break;
	case IF_PATTERN:
		dev_err(pcw->sd.dev, "IF Pattern not yet implemented\n");
		return -EINVAL;
	default:
		dev_err(pcw->sd.dev, "Invalid streaming source\n");
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops stream_src_ctrl_ops = {
	.s_ctrl = stream_src_s_ctrl,
};

static int sync_mode_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);

	if (!has_io_op(pcw, configure_sync_mode))
		return -EINVAL;

	if (ctrl->val < 0 || ctrl->val > 2) {
		dev_err(pcw->sd.dev, "Invalid sync mode\n");
		return -EINVAL;
	}
	return call_io_op(pcw, configure_sync_mode, ctrl->val);
}

static int s_io_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);

	if (!pcw->initialized)
		return 0;

	if (ctrl->id == PSEE_CID_SYNC_MODE)
		return sync_mode_s_ctrl(ctrl);

	return -EINVAL;
}

static const struct v4l2_ctrl_ops io_ctrl_ops = {
	.s_ctrl = s_io_ctrl,
};

static bool roi_ctrl_equal(const struct v4l2_ctrl *ctrl,
			   union v4l2_ctrl_ptr ptr1,
			   union v4l2_ctrl_ptr ptr2)
{
	struct roi_set *roi_set1 = (struct roi_set *)ptr1.p;
	struct roi_set *roi_set2 = (struct roi_set *)ptr2.p;
	int i;
	bool eq;

	eq = roi_set1->n == roi_set2->n;

	for (i = 0; (i < roi_set1->n) && eq; ++i) {
		struct roi *roi1 = &roi_set1->rois[i];
		struct roi *roi2 = &roi_set2->rois[i];

		eq &= roi1->x == roi2->x;
		eq &= roi1->y == roi2->y;
		eq &= roi1->width == roi2->width;
		eq &= roi1->height == roi2->height;
	}

	return eq;
}

static void roi_ctrl_init(const struct v4l2_ctrl *ctrl, u32 idx,
			  union v4l2_ctrl_ptr ptr)
{
	struct roi_set *roi_set = (struct roi_set *)ptr.p;

	roi_set->n = 0;
}

static void roi_ctrl_log(const struct v4l2_ctrl *ctrl)
{
	// TODO: how to log ?&
}

static int roi_ctrl_validate(const struct v4l2_ctrl *ctrl, union v4l2_ctrl_ptr ptr)
{
	struct roi_set *roi_set = (struct roi_set *)ctrl->p_new.p;
	int i;

	for (i = 0; i < roi_set->n; ++i) {
		struct roi *roi = &roi_set->rois[i];
		bool xvalid = (roi->x >= 0 && roi->x < 320 && roi->x + roi->width < 320);
		bool yvalid = (roi->y >= 0 && roi->y < 320 && roi->y + roi->height < 320);

		if (!xvalid || !yvalid)
			return 1;
	}

	return 0;
}

static const struct v4l2_ctrl_type_ops roi_type_ops = {
	.equal = roi_ctrl_equal,
	.init = roi_ctrl_init,
	.log = roi_ctrl_log,
	.validate = roi_ctrl_validate,
};

static bool roi_pixel_ctrl_equal(const struct v4l2_ctrl *ctrl,
				 union v4l2_ctrl_ptr ptr1,
				 union v4l2_ctrl_ptr ptr2)
{
	struct grid *grid1 = (struct grid *)ptr1.p;
	struct grid *grid2 = (struct grid *)ptr2.p;
	int i, j;
	bool eq;

	eq = grid1->width == grid2->width && grid1->height == grid2->height;

	for (i = 0; i < grid1->height; i++) {
		for (j = 0; j < grid1->width / 32; j++)
			eq &= grid1->rows[i].vectors[j] == grid2->rows[i].vectors[j];
	}

	return eq;
}

static void roi_pixel_ctrl_init(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr)
{
	struct grid *grid = (struct grid *)ptr.p;
	struct v4l2_mbus_framefmt format;
	u32 y;

	// TODO: get resolution from driver ?
	psee_get_format(ctrl_to_pcw(ctrl), &format);
	grid->width = format.width;
	grid->height = format.height;

	for (y = 0; y < grid->height; y++)  {
		memset(&grid->rows[y].vectors, 0xFF, (grid->width / 32));
		grid->rows[y].dirty = false;
	}
}

static void roi_pixel_ctrl_log(const struct v4l2_ctrl *ctrl)
{
	// TODO: how to log ?&
}

static int roi_pixel_ctrl_validate(const struct v4l2_ctrl *ctrl, union v4l2_ctrl_ptr ptr)
{
	struct psee_v4l2_ctrl_wrapper *pcw = ctrl_to_pcw(ctrl);
	struct grid *grid = (struct grid *)ctrl->p_new.p;
	struct v4l2_mbus_framefmt format;

	psee_get_format(pcw, &format);
	if (grid->width != format.width && grid->height != format.height)
		return 1;

	return 0;
}

static const struct v4l2_ctrl_type_ops roi_pixel_type_ops = {
	.equal = roi_pixel_ctrl_equal,
	.init = roi_pixel_ctrl_init,
	.log = roi_pixel_ctrl_log,
	.validate = roi_pixel_ctrl_validate,
};

static const char * const sync_mode_name[] = {
	"Standalone",
	"Master",
	"Slave",
};

static const struct v4l2_ctrl_config sync_mode_cfg = {
    .id            = PSEE_CID_SYNC_MODE,
    .name          = "sync_mode",
    .type          = V4L2_CTRL_TYPE_MENU,
    .min           = 0,
    .max           = ARRAY_SIZE(sync_mode_name) - 1,
    .def           = 0,
    .menu_skip_mask = 0,
    .qmenu          = sync_mode_name,
	.ops 			= &io_ctrl_ops,
};

struct v4l2_ctrl_config roi_roni = {
	.id = PSEE_CID_ROI_RONI,
	.ops = &esp_ctrl_ops,
	.name = "roi_roni",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

struct v4l2_ctrl_config roi_reset = {
	.id = PSEE_CID_ROI_RESET,
	.ops = &esp_ctrl_ops,
	.name = "roi_reset",
	.type = V4L2_CTRL_TYPE_BUTTON,
	.min = 0,
	.max = 0,
	.step = 0,
	.def = 0,
};

struct v4l2_ctrl_config roi_pixel_set = {
	.id = GENX320_CID_ROI_PIXEL_ARRAY,
	.ops = &esp_ctrl_ops,
	.type_ops = &roi_pixel_type_ops,
	.name = "pxl_set",
	.type = PSEE_CMPTYPE_ROI_PIXEL,
	.elem_size = sizeof(struct grid),
};

struct v4l2_ctrl_config roi_window_enable = {
	.id = PSEE_CID_ROI_ENABLE,
	.ops = &esp_ctrl_ops,
	.name = "roi_enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

struct v4l2_ctrl_config roi_window_set = {
	.id = PSEE_CID_ROI_WINDOW_SET,
	.ops = &esp_ctrl_ops,
	.type_ops = &roi_type_ops,
	.name = "roi_set",
	.type = PSEE_CMPTYPE_ROI,
	.elem_size = sizeof(struct roi_set),
};

struct v4l2_ctrl_config erc_enable_ctrl = {
	.id = PSEE_CID_ERC_ENABLE,
	.ops = &esp_ctrl_ops,
	.name = "erc_enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.def = 0,
	.step = 1,
};

static const char * const event_source_name[] = {
	"Pixel Array",
	"Time base",
	"Readout Pattern",
	"IF Pattern",
};

// struct v4l2_ctrl_config stream_src = {
//     .id = V4L2_CID_TEST_PATTERN,
//     .ops = stream_src_ops,
//     .type = V4L2_CTRL_TYPE_MENU,
//     .max = ARRAY_SIZE(event_source_name) - 1,
//     .def = PIXEL_ARRAY,
//     .menu_skip_mask = 0,
//     .qmenu = event_source_name,
// }

// TODO: get values from driver
#define ERC_REF_PERIOD_DEFAULT 100
#define ERC_TD_EVENT_COUNT_MAX 20000ll
#define ERC_TD_EVENT_COUNT_DEFAULT 1000

#define ERC_MIN_EVENT_RATE 0
#define ERC_MAX_EVENT_RATE \
	((ERC_TD_EVENT_COUNT_MAX * 1000000) / ERC_REF_PERIOD_DEFAULT)

#define ERC_DEFAULT_EVENT_RATE \
	((ERC_TD_EVENT_COUNT_DEFAULT * 1000000) / ERC_REF_PERIOD_DEFAULT)

struct v4l2_ctrl_config erc_rate_ctrl = {
	.id = PSEE_CID_ERC_RATE,
	.ops = &esp_ctrl_ops,
	.name = "erc_rate",
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.min = ERC_MIN_EVENT_RATE,
	.max = ERC_MAX_EVENT_RATE,
	.def = ERC_DEFAULT_EVENT_RATE,
	.step = 1,
};

static int get_num_biases(struct psee_v4l2_ctrl_wrapper *pcw)
{
	int id = 0;
	u32 num_bias = 0;

	while (call_esp_op(pcw, bias, get, id++) >= 0)
		num_bias++;

	return num_bias;
}

static int build_bias_ctrls(struct psee_v4l2_ctrl_wrapper *pcw, const struct v4l2_ctrl_ops *ops,
			    struct v4l2_ctrl_config *configs)
{
	int id = 0;
	u32 def, min, max, val;
	u32 num_bias = 0;
	static char *name;

	num_bias = get_num_biases(pcw);

	for (id = 0; id < num_bias; id++) {
		def = call_esp_op(pcw, bias, get_default, id);
		min = call_esp_op(pcw, bias, get_min, id);
		max = call_esp_op(pcw, bias, get_max, id);
		val = call_esp_op(pcw, bias, get, id);
		if (call_esp_op(pcw, bias, get_name, id, &name) < 0)
			return 0;

		configs[id].id = PSEE_CID_BIAS_CTRL + id;
		configs[id].ops = ops;
		configs[id].name = name;
		configs[id].type = V4L2_CTRL_TYPE_INTEGER;
		configs[id].def = def;
		configs[id].min = min;
		configs[id].max = max;
		configs[id].step = 1;
	}

	return num_bias;
}

int psee_get_format(struct psee_v4l2_ctrl_wrapper *pcw, struct v4l2_mbus_framefmt *format)
{
	call_core_op(pcw, get_width, &format->width);
	call_core_op(pcw, get_height, &format->height);

	return 0;
}

int psee_init_controls(struct psee_v4l2_ctrl_wrapper *pcw, const struct psee_ctrl_ops *ctrl_ops,
		       struct psee_ops *ops)
{
	struct psee_controls *controls = &pcw->controls;
	struct v4l2_ctrl_config *bias_ctrls;
	struct v4l2_ctrl_handler *hdl = &pcw->hdl;
	int id, num_bias;

	v4l2_ctrl_handler_init(hdl, 17);
	controls->dev_ctrl = *ctrl_ops;
	controls->dev_ctrl.hdl = pcw;
	pcw->ops = ops;

	if (!pcw->ops || !pcw->ops->esp)
		return -EINVAL;

	pcw->stream_ctrl = v4l2_ctrl_new_std_menu_items(
		hdl,
		&stream_src_ctrl_ops,
		V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(event_source_name) - 1,
		0, PIXEL_ARRAY, event_source_name);

	if (pcw->ops->io) {
		pcw->sync_ctrl = v4l2_ctrl_new_custom(hdl, &sync_mode_cfg, NULL);
	}

	if (pcw->ops->esp->roi_window) {
		v4l2_ctrl_new_custom(hdl, &roi_reset, NULL);
		v4l2_ctrl_new_custom(hdl, &roi_roni, NULL);
		pcw->roi_window_set = v4l2_ctrl_new_custom(hdl, &roi_window_set, NULL);
	}

	if (pcw->ops->esp->roi_pixel)
		v4l2_ctrl_new_custom(hdl, &roi_pixel_set, NULL);

	if (pcw->ops->esp->erc) {
		pcw->erc_enable = v4l2_ctrl_new_custom(hdl, &erc_enable_ctrl, NULL);
		pcw->erc_rate = v4l2_ctrl_new_custom(hdl, &erc_rate_ctrl, NULL);
	}

	if (pcw->ops->esp->bias) {
		num_bias = get_num_biases(pcw);
		bias_ctrls = kcalloc(num_bias, sizeof(struct v4l2_ctrl_config), GFP_KERNEL);
		build_bias_ctrls(pcw, &esp_ctrl_ops, bias_ctrls);

		for (id = 0; id < num_bias; id++)
			v4l2_ctrl_new_custom(hdl, &bias_ctrls[id], NULL);

		kfree(bias_ctrls);
	}

	pcw->sd.ctrl_handler = hdl;

	if (pcw->ops->esp->erc)
		v4l2_ctrl_cluster(2, &pcw->erc_enable);

	return 0;
}

