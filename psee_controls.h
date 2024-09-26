/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PSEE_CONTROLS_H
#define __PSEE_CONTROLS_H
#include <linux/compiler.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include "drivers/api.h"

#define RET_ON(operation) do { int r = operation; if (unlikely(r != 0)) return r; } while (0)

#define PSEE_CTRL_CLASS(CID) ((CID) & 0xF000)

#define PSEE_BIAS_CLASS 0x3000
#define PSEE_CID_BIAS_CTRL (V4L2_CID_USER_BASE  | PSEE_BIAS_CLASS)

#define PSEE_CMPTYPE_ROI (V4L2_CTRL_COMPOUND_TYPES + 101)
#define PSEE_CMPTYPE_ROI_PIXEL (V4L2_CTRL_COMPOUND_TYPES + 102)

#define PSEE_ROI_CLASS 0x4000
#define PSEE_CID_ROI_CTRL (V4L2_CID_USER_BASE  | PSEE_ROI_CLASS)

#define PSEE_CID_ROI_MODE (PSEE_CID_ROI_CTRL)
#define PSEE_CID_ROI_RESET (PSEE_CID_ROI_CTRL + 1)
#define PSEE_CID_ROI_WINDOW_APPEND (PSEE_CID_ROI_CTRL + 2)
#define PSEE_CID_ROI_WINDOW_UPDATE (PSEE_CID_ROI_CTRL + 3)
#define PSEE_CID_ROI_WINDOW_SET (PSEE_CID_ROI_CTRL + 4)
#define PSEE_CID_ROI_ENABLE (PSEE_CID_ROI_CTRL + 5)
#define PSEE_CID_ROI_RONI (PSEE_CID_ROI_CTRL + 6)

#define GENX320_CID_ROI_PIXEL_RESET (PSEE_CID_ROI_CTRL + 7)
#define GENX320_CID_ROI_PIXEL_ARRAY (PSEE_CID_ROI_CTRL + 8)

#define PSEE_ERC_CLASS 0x1000
#define PSEE_CID_ERC_CTRL (V4L2_CID_USER_BASE  | PSEE_ERC_CLASS)
#define PSEE_CID_ERC_RATE   (PSEE_CID_ERC_CTRL + 1)
#define PSEE_CID_ERC_ENABLE (PSEE_CID_ERC_CTRL + 2)

#define PSEE_SOURCE_CLASS 0x2000
#define PSEE_CID_STREAMING_SOURCE (V4L2_CID_USER_BASE | PSEE_SOURCE_CLASS)

#define PSEE_CID_SENSOR_ID ((V4L2_CID_USER_BASE | PSEE_SOURCE_CLASS) + 1)

struct psee_roi_master_ops {
	int (*init)(struct psee_controls *controls);
	int (*append)(struct psee_controls *controls, struct roi roi, bool single);
	int (*update)(struct psee_controls *controls, struct roi roi, u32 index);
	int (*set)(struct psee_controls *controls, struct roi *roi, u32 n);
	int (*reset)(struct psee_controls *controls);
	int (*enable_roni)(struct psee_controls *controls, bool en);
};

struct psee_roi_pixel_ops {
	int (*init)(struct psee_controls *controls);
	int (*reset)(struct psee_controls *controls);
	int (*get_pixel)(struct psee_controls *controls, u32 x, u32 y, bool *enabled);
	int (*set_pixel)(struct psee_controls *controls, u32 x, u32 y, bool enable);
	int (*set_array)(struct psee_controls *controls, struct grid *grid);
	int (*get_array)(struct psee_controls *controls, struct grid *grid);
};

struct psee_erc_ops {
	int (*init)(struct psee_controls *controls);
	int (*enable)(struct psee_controls *controls, u32 en);
	int (*s_rate)(struct psee_controls *controls, s64 rate);
};

struct psee_bias_ops {
	int (*init)(struct psee_controls *controls);

	int (*get_name)(struct psee_controls *controls, u32 id, char **name);
	int (*set)(struct psee_controls *controls, u32 id, u8 value);
	int (*get)(struct psee_controls *controls, u32 id);

	int (*get_max)(struct psee_controls *controls, u32 id);
	int (*get_min)(struct psee_controls *controls, u32 id);
	int (*get_default)(struct psee_controls *controls, u32 id);
};

struct psee_esp_ops {
	const struct psee_erc_ops *erc;
	const struct psee_bias_ops *bias;
	const struct psee_roi_master_ops *roi_window;
	const struct psee_roi_pixel_ops *roi_pixel;
};

struct psee_ops {
	const struct psee_esp_ops *esp;
};

struct psee_v4l2_ctrl_wrapper {
	struct device *dev;
	struct v4l2_subdev sd;
	struct psee_controls controls;
	struct psee_ops *ops;
	struct v4l2_ctrl *roi_window_set;

	struct {
		struct v4l2_ctrl *erc_enable;
		struct v4l2_ctrl *erc_rate;
	};

	struct v4l2_ctrl_handler hdl;
	bool initialized;
};

static inline struct psee_v4l2_ctrl_wrapper *ctrl_to_pcw(const struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct psee_v4l2_ctrl_wrapper, hdl);
}

static inline struct psee_v4l2_ctrl_wrapper *sd_to_pcw(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct psee_v4l2_ctrl_wrapper, sd);
}

static inline struct psee_v4l2_ctrl_wrapper *psee_ctrl_to_pcw(struct psee_controls *ctrls)
{
	return container_of(ctrls, struct psee_v4l2_ctrl_wrapper, controls);
}

#define has_esp(pcw) ((pcw)->ops && (pcw)->ops->esp)
#define has_esp_mod(pcw, mod) (has_esp(pcw) && (pcw)->ops->esp->mod)
#define has_esp_op(pcw, mod, op) (has_esp_mod(pcw, mod) && (pcw)->ops->esp->mod->op)
#define call_esp_op(pcw, mod, op, ...) (has_esp_op(pcw, mod, op) ? \
	(pcw)->ops->esp->mod->op(&((pcw)->controls), \
				 ##__VA_ARGS__) : -EINVAL \
)

int psee_init_controls(struct psee_v4l2_ctrl_wrapper *pcw, const struct psee_ctrl_ops *ctrl_ops,
		       struct psee_ops *ops);
#endif /* __PSEE_CONTROLS_H */
