// SPDX-License-Identifier: GPL-2.0-only
#include "../api.h"
#include "../common.h"
#include "genx320_registers.h"
#include "genx320_mipi.h"

bool __is_big_endian(void)
{
#if !defined(__BYTE_ORDER__) || !defined(__ORDER_BIG_ENDIAN__)
	#error "Unknown byte order, both __BYTE_ORDER__ and __ORDER_BIG_ENDIAN__ must be defined."
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return 1;
#else
	return 0;
#endif
}


int __genx320_check_boot(struct psee_controls *controls)
{
	u32 magic, chip_id;

	RET_ON(read_register(controls->dev_ctrl, chip_id, &chip_id));
	if (chip_id != 0xb0602003)
		return -ENODEV;

	RET_ON(read_register(controls->dev_ctrl, mbx_misc, &magic));
	if (magic != 0xcafebabe)
		return -EIO;

	return 0;
}

int genx320_set_event_format(struct psee_controls *controls, enum event_format fmt)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct core_config *core = &controls->core;
	u32 fmt_val = 0;

	RET_ON(__genx320_check_boot(controls));

	switch (fmt) {
	case EVENT_FORMAT_EVT21:
		fmt_val = 2;
		break;
	case EVENT_FORMAT_EVT2:
		fmt_val = 0;
		break;
	case EVENT_FORMAT_EVT3:
		fmt_val = 1;
		break;
	default:
		return -EINVAL;
	}

	core->format = fmt;

	RET_ON(write_field(ctrl, edf_control, format, fmt_val));
	RET_ON(write_field(ctrl, edf_control, endianness, __is_big_endian()));

	RET_ON(write_field(ctrl, edf_pipeline_control, bypass, fmt == EVENT_FORMAT_EVT21));
	RET_ON(write_field(ctrl, edf_pipeline_control, enable, 1));
	return 0;
}

int genx320_start_pixel_array(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;

	ro_readout_ctrl ro_readout_ctrl;
	ro_td_ctrl ro_td_ctrl;
	roi_ctrl roi_ctrl;

	RET_ON(read_register(ctrl, ro_readout_ctrl, &ro_readout_ctrl.raw));
	ro_readout_ctrl.ro_self_test_en = 0;
	ro_readout_ctrl.ro_digital_pipe_en = 1;
	RET_ON(write_register(ctrl, ro_readout_ctrl, ro_readout_ctrl.raw));

	RET_ON(read_register(ctrl, ro_td_ctrl, &ro_td_ctrl.raw));
	ro_td_ctrl.ro_td_ack_y_rstn = 1;
	ro_td_ctrl.ro_td_arb_y_rstn = 1;
	ro_td_ctrl.ro_td_addr_y_rstn = 1;
	ro_td_ctrl.ro_td_sendreq_y_rstn = 1;
	RET_ON(write_register(ctrl, ro_td_ctrl, ro_td_ctrl.raw));

	RET_ON(read_register(ctrl, roi_ctrl, &roi_ctrl.raw));
	roi_ctrl.px_sw_rstn = 1;
	roi_ctrl.roi_td_en = 1;
	RET_ON(write_register(ctrl, roi_ctrl, roi_ctrl.raw));
	return 0;
}

int genx320_start_ro_pattern(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	ro_readout_ctrl ro_readout_ctrl;
	ro_td_ctrl ro_td_ctrl;

	RET_ON(read_register(ctrl, ro_readout_ctrl, &ro_readout_ctrl.raw));
	ro_readout_ctrl.ro_self_test_en = 1;
	ro_readout_ctrl.ro_digital_pipe_en = 1;
	RET_ON(write_register(ctrl, ro_readout_ctrl, ro_readout_ctrl.raw));
	return 0;
}

int genx320_start_ts_pattern(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	ro_td_ctrl ro_td_ctrl;

	RET_ON(write_register(ctrl, ro_readout_ctrl, (u32)0));
	return 0;
}

int genx320_cpi_start_pattern(struct psee_controls *controls)
{
	RET_ON(__genx320_check_boot(controls));
	return 0;
}

int genx320_start_streaming(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct core_config *config = &controls->core;

	RET_ON(__genx320_check_boot(controls));

	switch (config->sensor_if) {
	case SENSOR_IF_MIPI:
		RET_ON(write_field(ctrl, mipi_csi_ctrl, enable, 1));
		break;
	case SENSOR_IF_PARALLEL:
		RET_ON(write_field(ctrl, cpi_pipeline_control, enable, 1));
		break;
	default: return -EINVAL;
	}

	RET_ON(write_field(ctrl, ro_lp_ctrl, lp_output_disable, 0));
	RET_ON(write_field(ctrl, ro_time_base_ctrl, time_base_enable, 1));

	switch (config->source) {
	case SENSOR_SOURCE_PIXEL_ARRAY:
		return genx320_start_pixel_array(controls);
	case SENSOR_SOURCE_RO_PATTERN:
		return genx320_start_ro_pattern(controls);
	case SENSOR_SOURCE_TS_PATTERN:
		return genx320_start_ts_pattern(controls);
	case SENSOR_SOURCE_IF_PATTERN:
		switch (config->sensor_if) {
		case SENSOR_IF_MIPI:
			return genx320_mipi_start_pattern(controls);
		case SENSOR_IF_PARALLEL:
			return genx320_cpi_start_pattern(controls);
		default: return -EINVAL;
		}
	default: return -EINVAL;
	}

	return 0;
}

int genx320_stop_streaming(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	ro_td_ctrl ro_td_ctrl;
	ro_lp_ctrl ro_lp_ctrl;

	RET_ON(__genx320_check_boot(controls));
	RET_ON(write_field(ctrl, roi_ctrl, px_sw_rstn, 0));

	RET_ON(read_register(ctrl, ro_td_ctrl, &ro_td_ctrl.raw));
	ro_td_ctrl.ro_td_ack_y_rstn = 0;
	ro_td_ctrl.ro_td_arb_y_rstn = 0;
	ro_td_ctrl.ro_td_addr_y_rstn = 0;
	ro_td_ctrl.ro_td_sendreq_y_rstn = 0;
	RET_ON(write_register(ctrl, ro_td_ctrl, ro_td_ctrl.raw));

	RET_ON(read_register(ctrl, ro_lp_ctrl, &ro_lp_ctrl.raw));
	ro_lp_ctrl.lp_output_disable = 1;
	ro_lp_ctrl.lp_keep_th = 0;
	RET_ON(write_register(ctrl, ro_lp_ctrl, ro_lp_ctrl.raw));

	msleep(1);

	RET_ON(write_field(ctrl, ro_time_base_ctrl, time_base_enable, 0));

	RET_ON(write_field(ctrl, mipi_csi_ctrl, enable, 0));

	return 0;
}

int genx320_wait_boot(struct psee_controls *controls)
{
	u32 retries = 0;
	u32 ret;

	while (retries++ < 10) {
		ret = __genx320_check_boot(controls);
		if (ret == -EIO) {
			msleep(1);
			continue;
		}

		return ret;

	}

	return -ETIMEDOUT;
}

int genx320_soft_reset(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	// dig_soft_reset
	RET_ON(write_field(ctrl, dig_soft_reset, digital_csr_srst, 1));
	// mbx/cpu_soft_rest
	RET_ON(write_field(ctrl, mbx_cpu_soft_reset, cpu_soft_reset, 1));
	msleep(10);
	// mbx/cpu_soft_rest
	RET_ON(write_field(ctrl, mbx_cpu_soft_reset, cpu_soft_reset, 0));

	return genx320_wait_boot(controls);
}


int genx320_get_width(struct psee_controls *controls, u32 *width)
{
	*width = 320;
	return 0;
}

int genx320_get_height(struct psee_controls *controls, u32 *height)
{
	*height = 320;
	return 0;
}
