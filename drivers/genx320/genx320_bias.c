// SPDX-License-Identifier: GPL-2.0-only

#include "../common.h"
#include "genx320_registers.h"
#include "genx320_bias.h"

struct bias_settings {
	char name[BIAS_NAME_MAX_SIZE];
	u32  addr;
	u8   min;
	u8   max;
	u8   def;
	u8   cur;
};

static struct bias_settings settings[] = {
	{
		.name = "bias_fo",
		.addr = bias0_fo_address,
		.min = 19,
		.max = 39,
		.def = 34,
	},
	{
		.name = "bias_hpf",
		.addr = bias0_hpf_address,
		.min = 0,
		.max = 127,
		.def = 40,
	},
	{
		.name = "bias_diff_on",
		.addr = bias0_diff_on_address,
		.min = 24,
		.max = 60,
		.def = 25,
	},
	{
		.name = "bias_diff",
		.addr = bias0_diff_address,
		.min = 41,
		.max = 51,
		.def = 51,
	},
	{
		.name = "bias_diff_off",
		.addr = bias0_diff_off_address,
		.min = 19,
		.max = 50,
		.def = 28,
	},
	{
		.name = "bias_refr",
		.addr = bias0_refr_address,
		.min = 0,
		.max = 127,
		.def = 10
	},
};

int genx320_bias_init(struct psee_controls *controls)
{
	u32 id;

	for (id = 0; id < ARRAY_SIZE(settings); id++)
		settings[id].cur = settings[id].def;

	return 0;
}

int genx320_bias_get_name(struct psee_controls *controls, u32 id, char **name)
{
	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;

	*name = settings[id].name;
	return 0;
}

int genx320_bias_set(struct psee_controls *controls, u32 id, u8 value)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	bias0_fo reg;

	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;

	RET_ON(ctrl.read_reg(ctrl.hdl, settings[id].addr, &reg.raw));
	reg.ctl = value;
	reg.single = 1;
	RET_ON(ctrl.write_reg(ctrl.hdl, settings[id].addr, reg.raw));
	settings[id].cur = value;
	return 0;
}

int genx320_bias_get(struct psee_controls *controls, u32 id)
{
	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;
	return settings[id].cur;
}

int genx320_bias_get_max(struct psee_controls *controls, u32 id)
{
	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;
	return settings[id].max;
}

int genx320_bias_get_min(struct psee_controls *controls, u32 id)
{
	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;
	return settings[id].min;
}

int genx320_bias_get_default(struct psee_controls *controls, u32 id)
{
	if (id >= ARRAY_SIZE(settings))
		return -EINVAL;
	return settings[id].def;
}
