/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PSEE_CONTROLS_H
#define __PSEE_CONTROLS_H
#include <linux/compiler.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define RET_ON(operation) do { int r = operation; if (unlikely(r != 0)) return r; } while (0)

struct psee_v4l2_ctrl_wrapper {
	struct device *dev;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
};

static inline struct psee_v4l2_ctrl_wrapper *sd_to_pcw(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct psee_v4l2_ctrl_wrapper, sd);
}

#endif /* __PSEE_CONTROLS_H */
