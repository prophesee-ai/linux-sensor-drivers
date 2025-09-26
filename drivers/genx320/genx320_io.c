// SPDX-License-Identifier: GPL-2.0-only
#include "../api.h"
#include "../common.h"
#include "genx320_registers.h"
#include "genx320_io.h"


int genx320_io_sync_mode(struct psee_controls *controls, enum sync_mode mode) {
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;

	u32 external = (mode != SYNC_MODE_STANDALONE);
	u32 master = (mode == SYNC_MODE_MASTER);

	RET_ON(write_field(ctrl, ro_time_base_ctrl, time_base_mode, external));
	RET_ON(write_field(ctrl, ro_time_base_ctrl, external_mode, master)); 
	RET_ON(write_field(ctrl, ro_time_base_ctrl, external_mode_enable, external));

	if (external)
	{
		if (master)
		{
			// set SYNCHRO IO to output mode
			RET_ON(write_field(ctrl, io_ctrl2, sync_enzi, 0));
			RET_ON(write_field(ctrl, io_ctrl2, sync_en, 0));
		}
		else
		{
			// set SYNCHRO IO to input mode
			RET_ON(write_field(ctrl, io_ctrl2, sync_enzi, 1));
			RET_ON(write_field(ctrl, io_ctrl2, sync_en, 1));
		}
	} 
	return 0;
}

