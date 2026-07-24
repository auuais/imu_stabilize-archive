// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Fri Jul 10 23:48:16 2026
// Host        : RMachine running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               <project>/ip/ddr4_sub64/ddr4_sub64_stub.v
// Design      : ddr4_sub64
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku15p-ffve1517-2-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* CORE_GENERATION_INFO = "DDR4_SDRAM, DDR4_SDRAM,{x_ipProduct=Vivado 2017.2.0,x_ipVendor=xilinx.com,x_ipLibrary=ip,x_ipName=DDR4_SDRAM,x_ipVersion=2.2, Controller_Type = DDR4_SDRAM, Time_Period = 1071, Input_Clock_Period = 4999, Memory_Type = Components, Memory_Part = MT40A512M16TB-062E, Ecc = false, Cas_Latency = 13, Cas_Write_Latency = 10, DQ_Width = 48, Chip_Select = true, Data_Mask = DM_NO_DBI, MEM_ADDR_ORDER = ROW_COLUMN_BANK,  Is_AXI_Enabled = false , Slot_cofiguration =  Single , Clamshell_cofiguration =  false ,IS_FASTER_SPEED_RAM = No, Is_custom_part = false, Memory_Voltage = 1.2V, Phy_Only = Complete_Memory_Controller, Debug_Port = Enable, Burst_Length = 8, System_Clock = Differential, AXI_Selection = false, AXI_Data_Width = 64,  AXI_ArbitrationScheme = RD_PRI_REG, AXI_Narrow_Burst = false, Simulation_Mode = BFM, Debug_Mode = Enable, Example_TG = SIMPLE_TG, Self_Refresh = false, Save_Restore = false, Enable_LVAUX = false,MicroBlaze_ECC = false,  Specify_MandD = false, CLKBOUT_MULT = 7, DIVCLK_DIVIDE = 1, CLKOUT0_DIVIDE = 6}" *) (* X_CORE_INFO = "ddr4_v2_2_28,Vivado 2025.2" *) (* dont_touch = "true" *) 
module ddr4_sub64(sys_rst, c0_sys_clk_p, c0_sys_clk_n, 
  c0_ddr4_act_n, c0_ddr4_adr, c0_ddr4_ba, c0_ddr4_bg, c0_ddr4_cke, c0_ddr4_odt, c0_ddr4_cs_n, 
  c0_ddr4_ck_t, c0_ddr4_ck_c, c0_ddr4_reset_n, c0_ddr4_dm_dbi_n, c0_ddr4_dq, c0_ddr4_dqs_c, 
  c0_ddr4_dqs_t, c0_init_calib_complete, c0_ddr4_ui_clk, c0_ddr4_ui_clk_sync_rst, dbg_clk, 
  dbg_rd_data_cmp, dbg_expected_data, dbg_cal_seq, dbg_cal_seq_cnt, dbg_cal_seq_rd_cnt, 
  dbg_rd_valid, dbg_cmp_byte, dbg_rd_data, dbg_cplx_config, dbg_cplx_status, dbg_io_address, 
  dbg_pllGate, dbg_phy2clb_fixdly_rdy_low, dbg_phy2clb_fixdly_rdy_upp, 
  dbg_phy2clb_phy_rdy_low, dbg_phy2clb_phy_rdy_upp, cal_r0_status, cal_post_status, 
  c0_ddr4_app_addr, c0_ddr4_app_cmd, c0_ddr4_app_en, c0_ddr4_app_hi_pri, 
  c0_ddr4_app_wdf_data, c0_ddr4_app_wdf_end, c0_ddr4_app_wdf_mask, c0_ddr4_app_wdf_wren, 
  c0_ddr4_app_rd_data, c0_ddr4_app_rd_data_end, c0_ddr4_app_rd_data_valid, 
  c0_ddr4_app_rdy, c0_ddr4_app_wdf_rdy, dbg_bus)
/* synthesis syn_black_box black_box_pad_pin="sys_rst,c0_sys_clk_p,c0_sys_clk_n,c0_ddr4_act_n,c0_ddr4_adr[16:0],c0_ddr4_ba[1:0],c0_ddr4_bg[0:0],c0_ddr4_cke[0:0],c0_ddr4_odt[0:0],c0_ddr4_cs_n[0:0],c0_ddr4_ck_t[0:0],c0_ddr4_ck_c[0:0],c0_ddr4_reset_n,c0_ddr4_dm_dbi_n[5:0],c0_ddr4_dq[47:0],c0_ddr4_dqs_c[5:0],c0_ddr4_dqs_t[5:0],c0_init_calib_complete,c0_ddr4_ui_clk_sync_rst,dbg_rd_data_cmp[63:0],dbg_expected_data[63:0],dbg_cal_seq[2:0],dbg_cal_seq_cnt[31:0],dbg_cal_seq_rd_cnt[7:0],dbg_rd_valid,dbg_cmp_byte[5:0],dbg_rd_data[63:0],dbg_cplx_config[15:0],dbg_cplx_status[1:0],dbg_io_address[27:0],dbg_pllGate,dbg_phy2clb_fixdly_rdy_low[19:0],dbg_phy2clb_fixdly_rdy_upp[19:0],dbg_phy2clb_phy_rdy_low[19:0],dbg_phy2clb_phy_rdy_upp[19:0],cal_r0_status[127:0],cal_post_status[8:0],c0_ddr4_app_addr[28:0],c0_ddr4_app_cmd[2:0],c0_ddr4_app_en,c0_ddr4_app_hi_pri,c0_ddr4_app_wdf_data[383:0],c0_ddr4_app_wdf_end,c0_ddr4_app_wdf_mask[47:0],c0_ddr4_app_wdf_wren,c0_ddr4_app_rd_data[383:0],c0_ddr4_app_rd_data_end,c0_ddr4_app_rd_data_valid,c0_ddr4_app_rdy,c0_ddr4_app_wdf_rdy,dbg_bus[511:0]" */
/* synthesis syn_force_seq_prim="c0_ddr4_ui_clk" */
/* synthesis syn_force_seq_prim="dbg_clk" */;
  input sys_rst;
  input c0_sys_clk_p;
  input c0_sys_clk_n;
  output c0_ddr4_act_n;
  output [16:0]c0_ddr4_adr;
  output [1:0]c0_ddr4_ba;
  output [0:0]c0_ddr4_bg;
  output [0:0]c0_ddr4_cke;
  output [0:0]c0_ddr4_odt;
  output [0:0]c0_ddr4_cs_n;
  output [0:0]c0_ddr4_ck_t;
  output [0:0]c0_ddr4_ck_c;
  output c0_ddr4_reset_n;
  inout [5:0]c0_ddr4_dm_dbi_n;
  inout [47:0]c0_ddr4_dq;
  inout [5:0]c0_ddr4_dqs_c;
  inout [5:0]c0_ddr4_dqs_t;
  output c0_init_calib_complete;
  output c0_ddr4_ui_clk /* synthesis syn_isclock = 1 */;
  output c0_ddr4_ui_clk_sync_rst;
  output dbg_clk /* synthesis syn_isclock = 1 */;
  output [63:0]dbg_rd_data_cmp;
  output [63:0]dbg_expected_data;
  output [2:0]dbg_cal_seq;
  output [31:0]dbg_cal_seq_cnt;
  output [7:0]dbg_cal_seq_rd_cnt;
  output dbg_rd_valid;
  output [5:0]dbg_cmp_byte;
  output [63:0]dbg_rd_data;
  output [15:0]dbg_cplx_config;
  output [1:0]dbg_cplx_status;
  output [27:0]dbg_io_address;
  output dbg_pllGate;
  output [19:0]dbg_phy2clb_fixdly_rdy_low;
  output [19:0]dbg_phy2clb_fixdly_rdy_upp;
  output [19:0]dbg_phy2clb_phy_rdy_low;
  output [19:0]dbg_phy2clb_phy_rdy_upp;
  output [127:0]cal_r0_status;
  output [8:0]cal_post_status;
  input [28:0]c0_ddr4_app_addr;
  input [2:0]c0_ddr4_app_cmd;
  input c0_ddr4_app_en;
  input c0_ddr4_app_hi_pri;
  input [383:0]c0_ddr4_app_wdf_data;
  input c0_ddr4_app_wdf_end;
  input [47:0]c0_ddr4_app_wdf_mask;
  input c0_ddr4_app_wdf_wren;
  output [383:0]c0_ddr4_app_rd_data;
  output c0_ddr4_app_rd_data_end;
  output c0_ddr4_app_rd_data_valid;
  output c0_ddr4_app_rdy;
  output c0_ddr4_app_wdf_rdy;
  output [511:0]dbg_bus;
endmodule
