/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __GENX320_REGISTERS_H
#define __GENX320_REGISTERS_H

#define PSEE_REGISTER_DEF(register_name, __address, reg_struct) \
	static const u32 register_name##_address = __address; \
	static const u32 register_name##_size = 4; \
	typedef union register_name##_reg { \
		u32 raw; \
		struct reg_struct; \
	} register_name

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

PSEE_REGISTER_DEF(edf_pipeline_control, 0x7000, {
	u32 enable :1;
	u32 drop_nbackpressure :1;
	u32 bypass :1;
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

#endif // __GENX320_REGISTERS_H
