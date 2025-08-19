/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __GENX320_REGISTERS_H
#define __GENX320_REGISTERS_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/errno.h>
#else // __KERNEL__
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int64_t s64;
#endif

#define PSEE_REGISTER_DEF(register_name, __address, reg_struct) \
	static const u32 register_name##_address = __address; \
	static const u32 register_name##_size = 4; \
	typedef union register_name##_reg { \
		u32 raw; \
		struct reg_struct; \
	} register_name

#define FIELD_MASK(register, field) ({ \
	union register##_reg __reg = {.raw = 0xFFFFFFFF}; \
	(u32)__reg.field; \
})

#define CHECK_SIZE(register, field, value) ({ \
	u32 __mask = FIELD_MASK(register, field); \
	u32 __value = value; \
	((__value & __mask) == __value); \
})

#define write_register(ctrl, register, __value) ({ \
	int rc = 0; \
	union register##_reg __reg = (union register##_reg)__value; \
	rc = ctrl.write_reg(ctrl.hdl, register##_address, __reg.raw); \
	rc; \
})

#define read_register(ctrl, register, pdst) ({ \
	int rc = 0; \
	union register##_reg __reg; \
	rc = ctrl.read_reg(ctrl.hdl, register##_address, &__reg.raw); \
	*(pdst) = __reg.raw; \
	rc; \
})

#define write_field(ctrl, register, field, __value) ({ \
	int rc = -EOVERFLOW; \
	union register##_reg __reg; \
	if (CHECK_SIZE(register, field, __value)) { \
		rc = ctrl.read_reg(ctrl.hdl, register##_address, &__reg.raw); \
		__reg.field = __value; \
		rc = rc < 0 ? rc : ctrl.write_reg(ctrl.hdl, register##_address, __reg.raw); \
	} \
	rc; \
})

#define __s(field, __value) \
	__temp_reg.field = __value;

#define write_fields(ctrl, register, initializer) ({ \
	int rc = 0; \
	union register##_reg __temp_reg; \
	rc = ctrl.read_reg(ctrl.hdl, register##_address, &__temp_reg.raw); \
	initializer; \
	rc = rc < 0 ? rc : ctrl.write_reg(ctrl.hdl, register##_address, __temp_reg.raw); \
	rc; \
})

#define read_field(ctrl, register, field, pdst) ({ \
	u32 rc = 0; \
	union register##_reg __reg; \
	rc = ctrl.read_reg(ctrl.hdl, register##_address, &__reg.raw); \
	if (rc >= 0) \
		*(pdst) = __reg.field; \
	rc; \
})

PSEE_REGISTER_DEF(roi_ctrl, 0x0000, {
	u32:1;
	u32 roi_td_en :1;
	u32:3;
	u32 td_shadow_trigger :1;
	u32 px_iphoto_en :1;
	u32 px_row_mon_rstn :1;
	u32:2;
	u32 px_sw_rstn :1;
	u32 px_roi_halt_programming :1;
	u32:20;
});

PSEE_REGISTER_DEF(chip_id, 0x0014, {
	u32 chip_id;
});

PSEE_REGISTER_DEF(roi_master_ctrl, 0x0034, {
	u32 master_en :1;
	u32 master_run :1;
	u32 master_mode :1;
	u32 win_nb :5;
	u32:8;
	u32 master_busy :1;
	u32 master_done :1;
});

PSEE_REGISTER_DEF(roi_master_chicken_bit, 0x0044, {
	u32 driver_register_if_en :1;
	u32 hold_time :5;
});

PSEE_REGISTER_DEF(dig_soft_reset, 0x001C, {
	u32 digital_csr_srst :1;
	u32 digital_pipe_srst :1;
	u32 analog_rstn :1;
	u32 pdl_override :1;
});

PSEE_REGISTER_DEF(ro_td_ctrl, 0x002C, {
	u32 ro_td_act_pdy_drive :3;
	u32 ro_td_act_pu_drive :4;
	u32 ro_td_sendreq_y_stat_en :1;
	u32 ro_td_sendreq_y_rstn :1;
	u32 ro_td_int_x_rstn :1;
	u32 ro_td_int_y_rstn :1;
	u32 ro_td_int_x_stat_en :1;
	u32 ro_td_int_y_stat_en :1;
	u32 ro_td_addr_y_stat_en :1;
	u32 ro_td_addr_y_rstn :1;
	u32 ro_td_ack_y_rstn :1;
	u32:1;
	u32 ro_td_arb_y_rstn :1;
	u32 ro_td_ack_y_set :1;
	u32 ro_td_int_x_act_pu :3;
	u32 ro_td_reqx_ctrllast_bypass :1;
});

PSEE_REGISTER_DEF(sram_initn, 0x00B8, {
	u32 afk_initn :1;
	u32 ehc_stc_initn :1;
	u32 erc_dl_initn :1;
	u32 erc_ilg_initn :1;
	u32 erc_tdrop_initn :1;
	u32 mipi_initn :1;
	u32 cpi_initn :1;
	u32 imem_initn :1;
	u32 dmem_initn :1;
	u32 rom_initn :1;
	u32:22;
});

PSEE_REGISTER_DEF(sram_pd1, 0x00C0, {
	u32 dmem_pd :1;
	u32 imem_pd :1;
	u32 rom_pd :1;
	u32 erc_dl_pd :1;
	u32 erc_ilg_pd :1;
	u32 erc_tdrop_pd :1;
	u32 mipi_pd :1;
	u32 cp_pd :1;
});

#define MAX_ROI_WINDOWS 18
PSEE_REGISTER_DEF(roi_win_array, 0x0400, {
	u32 roi_win_start :9;
	u32:7;
	u32 roi_win_end :9;
	u32:7;
});

PSEE_REGISTER_DEF(mbx_cpu_soft_reset, 0xF004, {
	u32 cpu_soft_reset :1;
	u32:31;
});

PSEE_REGISTER_DEF(sys_clk_ctrl, 0x0204, {
	u32 sys_clk_en :1;
	u32 sys_clk_switch :1;
	u32 phy_clk_off_count :6;
	u32 phy_clk_on_count :6;
	u32 phy_clk_div2 :1;
	u32 sys_clk_auto_mode :1;
});

PSEE_REGISTER_DEF(pll_ctrl, 0x0214, {
	u32 pl_enable: 1;
});

PSEE_REGISTER_DEF(evt_icn_clk_ctrl, 0x0210, {
	u32 evt_icn_clk_en :1;
	u32 evt_icn_clk_switch :1;
	u32 evt_icn_clk_div :8;
	u32 esp_clk_en :1;
	u32 ro_clk_en :1;
});

PSEE_REGISTER_DEF(io_ctrl2, 0x608, {
	u32 sync_en	:1;
 	u32:3;
 	u32 sync_enzi :1;
});

// authorized ranges never exceed 7 bits. Let's keep the ctl field this size
// for now
#define BIAS_REGISTER_DEF(bank, name, address) \
PSEE_REGISTER_DEF(bias##bank##_##name, address, { \
	u32 ctl :7; \
	u32:21; \
	u32 single :1; \
})

BIAS_REGISTER_DEF(0, pr, 0x1000);
BIAS_REGISTER_DEF(0, fo, 0x1004);
BIAS_REGISTER_DEF(0, fes, 0x1008);

BIAS_REGISTER_DEF(1, pr, 0x100C);
BIAS_REGISTER_DEF(1, fo, 0x1010);
BIAS_REGISTER_DEF(1, fes, 0x1014);

BIAS_REGISTER_DEF(0, hpf, 0x1100);
BIAS_REGISTER_DEF(0, diff_on, 0x1104);
BIAS_REGISTER_DEF(0, diff, 0x1108);
BIAS_REGISTER_DEF(0, diff_off, 0x110C);
BIAS_REGISTER_DEF(0, inv, 0x1110);
BIAS_REGISTER_DEF(0, refr, 0x1114);
BIAS_REGISTER_DEF(0, invp, 0x1118);
BIAS_REGISTER_DEF(0, req_pu, 0x111C);
BIAS_REGISTER_DEF(0, sm_pdy, 0x1120);
;
BIAS_REGISTER_DEF(1, hpf, 0x1124);
BIAS_REGISTER_DEF(1, diff_on, 0x1128);
BIAS_REGISTER_DEF(1, diff, 0x112C);
BIAS_REGISTER_DEF(1, diff_off, 0x1130);
BIAS_REGISTER_DEF(1, inv, 0x1134);
BIAS_REGISTER_DEF(1, refr, 0x1138);
BIAS_REGISTER_DEF(1, invp, 0x113C);
BIAS_REGISTER_DEF(1, req_pu, 0x1140);
BIAS_REGISTER_DEF(1, sm_pdy, 0x1144);

PSEE_REGISTER_DEF(edf_control, 0x7044, {
	u32 format :2;
	u32:2;
	u32 endianness :1;
});

PSEE_REGISTER_DEF(bgen, 0x1000, {
	u32 bias_ctrl :7;
	u32:9;
	u32 buf_stg :2;
	u32:1;
	u32 ibtype_sel :1;
	u32:4;
	u32 bias_en :1;
	u32 pull_sel :1;
	u32:2;
	u32 single :1;
});

PSEE_REGISTER_DEF(bgen_ctrl, 0x1208, {
	u32 burst_transfer_hv_bank_0 :1;
	u32 burst_transfer_hv_bank_1 :1;
	u32 burst_transfer_lv_bank_0 :1;
	u32 burst_transfer_lv_bank_1 :1;
	u32 bias_rstn_hv :1;
	u32 bias_rstn_lv :1;
});

PSEE_REGISTER_DEF(td_roi_x_array, 0x2000, {
	u32 value;
});

PSEE_REGISTER_DEF(td_roi_y_array, 0x3000, {
	u32 value;
});

PSEE_REGISTER_DEF(erc_ref_period_flavor, 0x6030, {
	u32 reference_period : 10;
	u32:6;
	u32 avg_drop_rate_delayed: 1;
});

PSEE_REGISTER_DEF(erc_td_target_event_count, 0x602C, {
	u32 val;
});

PSEE_REGISTER_DEF(erc_ahvt_dropping_control, 0x6014, {
	u32 h_dropping_en :1;
	u32 v_dropping_en :1;
	u32 t_dropping_en :1;
	u32 t_dropping_lut_en :1;
	u32 drop_all_td_when_drop_geq : 10;
	u32:17;
	u32 status: 1;
});

PSEE_REGISTER_DEF(erc_pipeline_control, 0x6000, {
	u32 enable :1;
	u32 drop_nbackpressure :1;
	u32 bypass :1;
});

PSEE_REGISTER_DEF(erc_delay_fifo_flush_and_bypass, 0x60A0, {
	u32 en :1;
	u32:30;
	u32 status :1;
});

PSEE_REGISTER_DEF(erc_monitoring_event_control, 0x6034, {
	u32 first_module_tag_en :1;
	u32 avg_drop_rate_en :1;
	u32 in_td_cnt_en :1;
	u32 df_td_vect_drop_cnt_en :1;
	u32 df_non_td_vect_drop_cnt_en :1;
	u32 alldr_evt_drop_cnt_en :1;
	u32 hdr_evt_drop_cnt_en :1;
	u32 vdr_evt_drop_cnt_en :1;
	u32 tdr_evt_drop_cnt_en :1;
	u32 erc_td_evt_cnt_en :1;
	u32 last_module_tag_en :1;
});

PSEE_REGISTER_DEF(erc_reset_tdrop_counter_on_mtag_first, 0x60B4, {
	u32 en :1;
});

PSEE_REGISTER_DEF(edf_pipeline_control, 0x7000, {
	u32 enable :1;
	u32 drop_nbackpressure :1;
	u32 bypass :1;
});

PSEE_REGISTER_DEF(edf_event_injection, 0x7048, {
	u32 sysmon_end_of_frame_en :1;
	u32:31;
});

PSEE_REGISTER_DEF(edf_output_interface_control, 0x704C, {
	u32:4;
	u32 start_of_frame_timeout :12;
	u32:16;
});

PSEE_REGISTER_DEF(edf_external_output_adapter, 0x7100, {
	u32 qos_timeout :16;
	u32 atomic_qos_mode :1;
});

PSEE_REGISTER_DEF(cpi_pipeline_control, 0x8000, {
	u32 enable :1;
	u32 drop_nbackpressure :1;
	u32:1;
	u32 output_fifo_bypass :1;
	u32 output_data_format :1;
	u32 output_if_mode :1;
	u32 output_width :1;
	u32 packed_fixed_size_enable: 1;
	u32 packed_fixed_rate_enable: 1;
	u32 frame_fixed_size_enable: 1;
	u32 clk_out_en: 1;
	u32 clk_control_inversion: 1;
	u32 clk_out_gating_enable: 1;
	u32 clk_timeout: 8;
	u32 packet_pad_empty_enable :1;
	u32 hot_disable_enable :1;
});

PSEE_REGISTER_DEF(ro_readout_ctrl, 0x9000, {
	u32 ro_test_pixel_mux_en :1;
	u32 ro_self_test_en :1;
	u32 cpm_record_mode_en :1;
	u32 ro_analog_pipe_en :1;
	u32 erc_self_test_en :1;
	u32 ro_inv_pol_td :1;
	u32 ro_flip_x :1;
	u32 ro_flip_y :1;
	u32:1;
	u32 ro_digital_pipe_en :1;
	u32 ro_avoid_bress_td :1;
	u32:1;
	u32 drop_en :1;
	u32 drop_on_full_en :1;
	u32 delay_ro_td_int_x_act_fal : 4;
	u32 delay_ro_td_int_x_act_ris : 4;
});

PSEE_REGISTER_DEF(ro_time_base_ctrl, 0x9008, {
	u32 time_base_enable :1;
	u32 time_base_mode :1;
	u32 external_mode :1;
	u32 external_mode_enable :1;
	u32 us_counter_max :7;
	u32 th_every_64us_en :1;
	u32:4;
	u32 time_base_srst :1;
});
PSEE_REGISTER_DEF(ro_lp_ctrl, 0x9028, {
	u32 lp_cnt_en :1;
	u32 lp_output_disable :1;
	u32 lp_keep_th :1;
});

PSEE_REGISTER_DEF(mipi_csi_ctrl, 0xB000, {
	u32 enable :1;
	u32 empty :1;
	u32 busy :1;
	u32 frame_sync_en :1;
	u32 line_sync_en :1;
	u32:3;
	u32 channel :2;
	u32 data_type :6;
	u32 pkt_size :14;
});

PSEE_REGISTER_DEF(mipi_csi_frame_ctrl, 0xB010, {
	u32 pkt_timeout_en :1;
	u32 pkt_fix_rate_en :1;
	u32 pkt_fix_size_en :1;
	u32 frame_fix_rate_en :1;
	u32 frame_fix_size_en :1;
	u32 fix_rate_empty_pkt :1;
	u32:26;
});

PSEE_REGISTER_DEF(mipi_csi_bl_frame, 0xB024, {
	u32 val :24;
	u32:6;
	u32 ck_lane_hs :1;
	u32 enable :1;
});

PSEE_REGISTER_DEF(mipi_csi_stat_ctrl, 0xB080, {
	u32 enable :1;
	u32 trigger :1;
	u32 clear :1;
});

PSEE_REGISTER_DEF(mipi_csi_stat_frame_cnt, 0xB084, {
	u32 val;
});

PSEE_REGISTER_DEF(mipi_csi_stat_byte_cnt, 0xB088, {
	u32 val;
});

PSEE_REGISTER_DEF(mipi_csi_stat_pad_cnt, 0xB08C, {
	u32 val;
});

PSEE_REGISTER_DEF(mipi_csi_stat_pkt_cnt, 0xB090, {
	u32 val;
});

PSEE_REGISTER_DEF(mipi_csi_stat_inc_pkt_cnt, 0xB094, {
	u32 val;
});

PSEE_REGISTER_DEF(mipi_csi_stat_frame_period, 0xB098, {
	u32 val;
});

PSEE_REGISTER_DEF(mbx_misc, 0xF010, {
	u32 misc;
});

#endif // __GENX320_REGISTERS_H
