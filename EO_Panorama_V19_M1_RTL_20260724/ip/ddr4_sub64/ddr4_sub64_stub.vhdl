-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
-- Date        : Fri Jul 10 23:48:16 2026
-- Host        : RMachine running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               <project>/ip/ddr4_sub64/ddr4_sub64_stub.vhdl
-- Design      : ddr4_sub64
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xcku15p-ffve1517-2-i
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ddr4_sub64 is
  Port ( 
    sys_rst : in STD_LOGIC;
    c0_sys_clk_p : in STD_LOGIC;
    c0_sys_clk_n : in STD_LOGIC;
    c0_ddr4_act_n : out STD_LOGIC;
    c0_ddr4_adr : out STD_LOGIC_VECTOR ( 16 downto 0 );
    c0_ddr4_ba : out STD_LOGIC_VECTOR ( 1 downto 0 );
    c0_ddr4_bg : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_cke : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_odt : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_cs_n : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_ck_t : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_ck_c : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_reset_n : out STD_LOGIC;
    c0_ddr4_dm_dbi_n : inout STD_LOGIC_VECTOR ( 5 downto 0 );
    c0_ddr4_dq : inout STD_LOGIC_VECTOR ( 47 downto 0 );
    c0_ddr4_dqs_c : inout STD_LOGIC_VECTOR ( 5 downto 0 );
    c0_ddr4_dqs_t : inout STD_LOGIC_VECTOR ( 5 downto 0 );
    c0_init_calib_complete : out STD_LOGIC;
    c0_ddr4_ui_clk : out STD_LOGIC;
    c0_ddr4_ui_clk_sync_rst : out STD_LOGIC;
    dbg_clk : out STD_LOGIC;
    dbg_rd_data_cmp : out STD_LOGIC_VECTOR ( 63 downto 0 );
    dbg_expected_data : out STD_LOGIC_VECTOR ( 63 downto 0 );
    dbg_cal_seq : out STD_LOGIC_VECTOR ( 2 downto 0 );
    dbg_cal_seq_cnt : out STD_LOGIC_VECTOR ( 31 downto 0 );
    dbg_cal_seq_rd_cnt : out STD_LOGIC_VECTOR ( 7 downto 0 );
    dbg_rd_valid : out STD_LOGIC;
    dbg_cmp_byte : out STD_LOGIC_VECTOR ( 5 downto 0 );
    dbg_rd_data : out STD_LOGIC_VECTOR ( 63 downto 0 );
    dbg_cplx_config : out STD_LOGIC_VECTOR ( 15 downto 0 );
    dbg_cplx_status : out STD_LOGIC_VECTOR ( 1 downto 0 );
    dbg_io_address : out STD_LOGIC_VECTOR ( 27 downto 0 );
    dbg_pllGate : out STD_LOGIC;
    dbg_phy2clb_fixdly_rdy_low : out STD_LOGIC_VECTOR ( 19 downto 0 );
    dbg_phy2clb_fixdly_rdy_upp : out STD_LOGIC_VECTOR ( 19 downto 0 );
    dbg_phy2clb_phy_rdy_low : out STD_LOGIC_VECTOR ( 19 downto 0 );
    dbg_phy2clb_phy_rdy_upp : out STD_LOGIC_VECTOR ( 19 downto 0 );
    cal_r0_status : out STD_LOGIC_VECTOR ( 127 downto 0 );
    cal_post_status : out STD_LOGIC_VECTOR ( 8 downto 0 );
    c0_ddr4_app_addr : in STD_LOGIC_VECTOR ( 28 downto 0 );
    c0_ddr4_app_cmd : in STD_LOGIC_VECTOR ( 2 downto 0 );
    c0_ddr4_app_en : in STD_LOGIC;
    c0_ddr4_app_hi_pri : in STD_LOGIC;
    c0_ddr4_app_wdf_data : in STD_LOGIC_VECTOR ( 383 downto 0 );
    c0_ddr4_app_wdf_end : in STD_LOGIC;
    c0_ddr4_app_wdf_mask : in STD_LOGIC_VECTOR ( 47 downto 0 );
    c0_ddr4_app_wdf_wren : in STD_LOGIC;
    c0_ddr4_app_rd_data : out STD_LOGIC_VECTOR ( 383 downto 0 );
    c0_ddr4_app_rd_data_end : out STD_LOGIC;
    c0_ddr4_app_rd_data_valid : out STD_LOGIC;
    c0_ddr4_app_rdy : out STD_LOGIC;
    c0_ddr4_app_wdf_rdy : out STD_LOGIC;
    dbg_bus : out STD_LOGIC_VECTOR ( 511 downto 0 )
  );

  attribute CORE_GENERATION_INFO : string;
  attribute CORE_GENERATION_INFO of ddr4_sub64 : entity is "DDR4_SDRAM, DDR4_SDRAM,{x_ipProduct=Vivado 2017.2.0,x_ipVendor=xilinx.com,x_ipLibrary=ip,x_ipName=DDR4_SDRAM,x_ipVersion=2.2, Controller_Type = DDR4_SDRAM, Time_Period = 1071, Input_Clock_Period = 4999, Memory_Type = Components, Memory_Part = MT40A512M16TB-062E, Ecc = false, Cas_Latency = 13, Cas_Write_Latency = 10, DQ_Width = 48, Chip_Select = true, Data_Mask = DM_NO_DBI, MEM_ADDR_ORDER = ROW_COLUMN_BANK,  Is_AXI_Enabled = false , Slot_cofiguration =  Single , Clamshell_cofiguration =  false ,IS_FASTER_SPEED_RAM = No, Is_custom_part = false, Memory_Voltage = 1.2V, Phy_Only = Complete_Memory_Controller, Debug_Port = Enable, Burst_Length = 8, System_Clock = Differential, AXI_Selection = false, AXI_Data_Width = 64,  AXI_ArbitrationScheme = RD_PRI_REG, AXI_Narrow_Burst = false, Simulation_Mode = BFM, Debug_Mode = Enable, Example_TG = SIMPLE_TG, Self_Refresh = false, Save_Restore = false, Enable_LVAUX = false,MicroBlaze_ECC = false,  Specify_MandD = false, CLKBOUT_MULT = 7, DIVCLK_DIVIDE = 1, CLKOUT0_DIVIDE = 6}";
  attribute dont_touch : string;
  attribute dont_touch of ddr4_sub64 : entity is "true";
end ddr4_sub64;

architecture stub of ddr4_sub64 is
  attribute syn_black_box : boolean;
  attribute black_box_pad_pin : string;
  attribute syn_black_box of stub : architecture is true;
  attribute black_box_pad_pin of stub : architecture is "sys_rst,c0_sys_clk_p,c0_sys_clk_n,c0_ddr4_act_n,c0_ddr4_adr[16:0],c0_ddr4_ba[1:0],c0_ddr4_bg[0:0],c0_ddr4_cke[0:0],c0_ddr4_odt[0:0],c0_ddr4_cs_n[0:0],c0_ddr4_ck_t[0:0],c0_ddr4_ck_c[0:0],c0_ddr4_reset_n,c0_ddr4_dm_dbi_n[5:0],c0_ddr4_dq[47:0],c0_ddr4_dqs_c[5:0],c0_ddr4_dqs_t[5:0],c0_init_calib_complete,c0_ddr4_ui_clk,c0_ddr4_ui_clk_sync_rst,dbg_clk,dbg_rd_data_cmp[63:0],dbg_expected_data[63:0],dbg_cal_seq[2:0],dbg_cal_seq_cnt[31:0],dbg_cal_seq_rd_cnt[7:0],dbg_rd_valid,dbg_cmp_byte[5:0],dbg_rd_data[63:0],dbg_cplx_config[15:0],dbg_cplx_status[1:0],dbg_io_address[27:0],dbg_pllGate,dbg_phy2clb_fixdly_rdy_low[19:0],dbg_phy2clb_fixdly_rdy_upp[19:0],dbg_phy2clb_phy_rdy_low[19:0],dbg_phy2clb_phy_rdy_upp[19:0],cal_r0_status[127:0],cal_post_status[8:0],c0_ddr4_app_addr[28:0],c0_ddr4_app_cmd[2:0],c0_ddr4_app_en,c0_ddr4_app_hi_pri,c0_ddr4_app_wdf_data[383:0],c0_ddr4_app_wdf_end,c0_ddr4_app_wdf_mask[47:0],c0_ddr4_app_wdf_wren,c0_ddr4_app_rd_data[383:0],c0_ddr4_app_rd_data_end,c0_ddr4_app_rd_data_valid,c0_ddr4_app_rdy,c0_ddr4_app_wdf_rdy,dbg_bus[511:0]";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of stub : architecture is "ddr4_v2_2_28,Vivado 2025.2";
begin
end;
