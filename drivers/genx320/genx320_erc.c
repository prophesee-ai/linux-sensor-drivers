// SPDX-License-Identifier: GPL-2.0-only
#include "../common.h"
#include "genx320_registers.h"
#include "genx320_erc.h"
#include "genx320_sram.h"

static u32 event_rate_to_event_count(u64 rate, u32 ref_period)
{
	return (rate * ref_period) / 1000000;
}

static u64 event_count_to_event_rate(u32 count, u32 ref_period)
{
	return (count * 1000000) / ref_period;
}

static int genx320_erc_set_evt_rate(struct psee_controls *controls, u32 ref_period,
				    u32 td_target_vx_cnt, u32 adr_delayed, u32 dfifo_non_td_area)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	u32 erc_dl_ok = is_powered_up_dyn(ctrl, erc_dl);

	if (adr_delayed && erc_dl_ok)
		RET_ON(write_field(ctrl, erc_delay_fifo_flush_and_bypass, en, 0));

	if (erc_dl_ok) {
		erc_ref_period_flavor rpf = {
			.avg_drop_rate_delayed = adr_delayed,
			.reference_period = ref_period,
		};
		RET_ON(write_register(ctrl, erc_ref_period_flavor, rpf));
	}
	return 0;
}

static int genx320_erc_activate_dyn(struct psee_controls *controls, u32 td_target_cnt)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct erc_config *config = &controls->erc;
	erc_monitoring_event_control mec = {0};
	erc_ahvt_dropping_control dc = {0};
	u32 status;

	// First force hold mode to flush ERC in case it is processing an event period
	erc_pipeline_control pctrl = {
		.enable = 0,
		.drop_nbackpressure = 0,
		.bypass = 0,
	};

	RET_ON(write_register(ctrl, erc_pipeline_control, pctrl));

	// Switch to bypass mode while setting configuration
	pctrl.bypass = 1;
	pctrl.enable = 1;
	RET_ON(write_register(ctrl, erc_pipeline_control, pctrl));

	// SRAM dfifo powerup
	RET_ON(write_field(ctrl, sram_initn, erc_dl_initn, 1));
	RET_ON(write_field(ctrl, sram_pd1, erc_dl_pd, 0));

	RET_ON(genx320_erc_set_evt_rate(controls, config->reference_period, td_target_cnt, 1, 28));

	mec.avg_drop_rate_en = 1;
	mec.in_td_cnt_en = 1;
	mec.erc_td_evt_cnt_en = 1;
	RET_ON(write_register(ctrl, erc_monitoring_event_control, mec));

	RET_ON(read_field(ctrl, erc_ahvt_dropping_control, status, &status));
	if (!status)
		return -EIO;

	dc.t_dropping_en = 1;
	dc.drop_all_td_when_drop_geq = 512;
	RET_ON(write_register(ctrl, erc_ahvt_dropping_control, dc));

	// Do not reset tdrop counter between event periods, since it mostly
	// preserves events at the beginning of the lines.
	RET_ON(write_field(ctrl, erc_reset_tdrop_counter_on_mtag_first, en, 0));

	// disable ERC bypass
	RET_ON(write_field(ctrl, erc_pipeline_control, bypass, 0));

	return 0;
}

static int genx320_erc_set_cd_event_count(struct psee_controls *controls, u32 count)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct erc_config *config = &controls->erc;
	int rc = 0;

	if (count > ERC_TD_EVENT_COUNT_MAX)
		return -EINVAL;

	RET_ON(write_register(ctrl, erc_td_target_event_count, count));
	config->td_target_event_count = count;
	return rc;
}

int genx320_erc_init(struct psee_controls *controls)
{
	struct erc_config *config = &controls->erc;
	// Default ERC configuration set to
	config->td_target_event_count = ERC_TD_EVENT_COUNT_DEFAULT;
	config->reference_period = ERC_REF_PERIOD_DEFAULT;

	return 0;
}

int genx320_erc_set_event_rate(struct psee_controls *controls, s64 rate)
{
	struct erc_config *config = &controls->erc;
	u32 event_count = event_rate_to_event_count(rate, config->reference_period);

	return genx320_erc_set_cd_event_count(controls, event_count);
}

int genx320_erc_enable(struct psee_controls *controls, u32 en)
{
	struct psee_ctrl_ops ctrl = controls->dev_ctrl;
	struct erc_config *config = &controls->erc;

	RET_ON(write_field(ctrl, erc_ahvt_dropping_control, t_dropping_en, !!en));
	if (en) {
		RET_ON(genx320_erc_set_cd_event_count(controls, config->td_target_event_count));
		RET_ON(genx320_erc_activate_dyn(controls, config->td_target_event_count));
	}

	return 0;
}

