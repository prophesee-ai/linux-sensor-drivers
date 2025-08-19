// SPDX-License-Identifier: GPL-2.0-only
#include "../api.h"
#include "../common.h"
#include "genx320_registers.h"
#include "genx320_mipi.h"

int genx320_mipi_configure_csi2_freq(struct psee_controls *controls, u32 bit_rate)
{
	// TODO; 800 is the reset configuration
	if (bit_rate != 800)
		return -EINVAL;

	return 0;
}

int genx320_mipi_set_packet_config(struct psee_controls *controls, enum mipi_frame_format fmt)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct mipi_config *mipi = &controls->mipi;

	mipi_csi_ctrl mipi_csi_ctrl;
	mipi_csi_frame_ctrl mipi_csi_frame_ctrl;
	edf_output_interface_control edf_output_interface_control;
	edf_external_output_adapter edf_external_output_adapter;
	sram_initn sram_initn;
	sram_pd1 sram_pd1;

	// TODO: support other configurations
	if (fmt != VARIABLE_SIZE)
		return -EINVAL;

	mipi->format = fmt;

	RET_ON(read_register(ctrl, mipi_csi_frame_ctrl, &mipi_csi_frame_ctrl.raw));
	mipi_csi_frame_ctrl.pkt_timeout_en = 0;
	mipi_csi_frame_ctrl.pkt_fix_rate_en = 0;
	mipi_csi_frame_ctrl.pkt_fix_size_en = 0;
	mipi_csi_frame_ctrl.frame_fix_rate_en = 0;
	mipi_csi_frame_ctrl.frame_fix_size_en = 0;
	mipi_csi_frame_ctrl.fix_rate_empty_pkt = 0;
	RET_ON(write_register(ctrl, mipi_csi_frame_ctrl, mipi_csi_frame_ctrl.raw));

	RET_ON(read_register(ctrl, mipi_csi_ctrl, &mipi_csi_ctrl.raw));
	mipi_csi_ctrl.pkt_size = 0x1000;
	RET_ON(write_register(ctrl, mipi_csi_ctrl, mipi_csi_ctrl.raw));

	RET_ON(read_register(ctrl, edf_output_interface_control,
			    &edf_output_interface_control.raw));

	edf_output_interface_control.start_of_frame_timeout = 0xFA;

	RET_ON(write_register(ctrl, edf_output_interface_control,
			     edf_output_interface_control.raw));

	RET_ON(read_register(ctrl, edf_external_output_adapter,
			    &edf_external_output_adapter.raw));

	edf_external_output_adapter.qos_timeout = 0xFFFF;
	edf_external_output_adapter.atomic_qos_mode = 0;

	RET_ON(write_register(ctrl, edf_external_output_adapter,
			     edf_external_output_adapter.raw));

	RET_ON(read_register(ctrl, sram_initn, &sram_initn.raw));
	sram_initn.mipi_initn = 1;
	RET_ON(write_register(ctrl, sram_initn, sram_initn.raw));

	RET_ON(read_register(ctrl, sram_pd1, &sram_pd1.raw));
	sram_pd1.mipi_pd = 0;
	RET_ON(write_register(ctrl, sram_pd1, sram_pd1.raw));

	mipi_csi_ctrl.enable = 1;
	RET_ON(write_register(ctrl, mipi_csi_ctrl, mipi_csi_ctrl.raw));

	RET_ON(write_register(ctrl, mipi_csi_bl_frame, 0x80000680));

	return 0;
}

int genx320_mipi_start_pattern(struct psee_controls *controls)
{
	return 0;
}

int genx320_mipi_configure(struct psee_controls *controls)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct core_config *core = &controls->core;
	struct mipi_config *mipi = &controls->mipi;

	if (mipi->num_lanes != 1)
		return -EINVAL;

	core->sensor_if = SENSOR_IF_MIPI;
	RET_ON(genx320_mipi_set_packet_config(controls, mipi->format));
	RET_ON(genx320_mipi_configure_csi2_freq(controls, mipi->bit_rate));

	if (mipi->stats_en)
		RET_ON(write_field(ctrl, mipi_csi_stat_ctrl, enable, 1));

	RET_ON(write_field(ctrl, edf_event_injection, sysmon_end_of_frame_en, mipi->eof_marker));

	return 0;
}

