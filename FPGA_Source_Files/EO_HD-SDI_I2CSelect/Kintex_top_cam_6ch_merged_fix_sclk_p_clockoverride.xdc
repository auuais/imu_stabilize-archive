# --------------------------------------------------------------------
# Kintex_top_cam_6ch (6 camera mux) - merged constraints
# Generated: 2026-01-03
#
# Sources merged:
#   - Kintex_top_0cam_ch1_1201.xdc  (CAM0 pins)
#   - Kintex_top_1cam_ch1_1202.xdc  (CAM1 pins + TRIG_IN1)
#   - Kintex_top_2cam_ch1_1202.xdc  (CAM2 pins + TRIG_IN2)
#   - Kintex_top_3cam_ch1_1202.xdc  (CAM3 pins + TRIG_IN3)
#   - Kintex_top_4cam_ch1_1202.xdc  (CAM4 pins + TRIG_IN4)
#   - Kintex_top_5cam_ch1_1202.xdc  (CAM5 pins + TRIG_IN5 + IEG0/IEG1 outputs + STROBE_OUT0)
#   - Kintex_top_i2C_test_1211.xdc  (I2C pins)
#
# NOTE (conflicts resolved for a single-file build):
#   * CAM0_PCLK uses PACKAGE_PIN C5 (from CAM0 XDC).
#   * SCLK_IN from the I2C XDC originally also used C5. To avoid conflict, this merged XDC
#     places SCLK_IN on P14 (the board clock pin commented in the original XDC headers).
#   * SDA/SCL from I2C use E19/E18. These conflict with IEG0_DOUT[1:0] in the original IEG mapping.
#     To keep I2C active, IEG0_DOUT[1:0] are re-mapped to M14/N15 (IOSTANDARD set to LVCMOS33).
#     If you need full IEG0_DOUT[19:0] on the original connector, revert these two pins back to:
#       IEG0_DOUT[0] -> E18 (LVCMOS18)
#       IEG0_DOUT[1] -> E19 (LVCMOS18)
#     and then move SDA/SCL to other suitable pins on your board.
# --------------------------------------------------------------------


#####################################################################
# Global control / clocks
#####################################################################

#set_property PACKAGE_PIN N14 [get_ports FPGA_RESET]
#set_property IOSTANDARD LVCMOS18 [get_ports FPGA_RESET]

# I2C controller reference clock input
#set_property PACKAGE_PIN P14 [get_ports SCLK_IN]
#set_property IOSTANDARD LVCMOS33 [get_ports SCLK_IN]
#create_clock -period 10.000 -name SCLK_IN [get_ports SCLK_IN]

#####################################################################
# I2C (cam_select)
#####################################################################

set_property PACKAGE_PIN G11 [get_ports SCL]
set_property IOSTANDARD LVCMOS33 [get_ports SCL]
#set_property PULLUP true         [get_ports SCL]

set_property PACKAGE_PIN F11 [get_ports SDA]
set_property IOSTANDARD LVCMOS33 [get_ports SDA]
#set_property PULLUP true         [get_ports SDA]


#####################################################################
# Camera inputs (CAM0~CAM5)
#####################################################################

# --- CAM0 ---
set_property PACKAGE_PIN C5 [get_ports CAM0_PCLK]
set_property IOSTANDARD LVCMOS33 [get_ports CAM0_PCLK]
set_property PACKAGE_PIN A6 [get_ports {CAM0_YOUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[0]}]
set_property PACKAGE_PIN B6 [get_ports {CAM0_YOUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[1]}]
set_property PACKAGE_PIN A3 [get_ports {CAM0_YOUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[2]}]
set_property PACKAGE_PIN A4 [get_ports {CAM0_YOUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[3]}]
set_property PACKAGE_PIN B4 [get_ports {CAM0_YOUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[4]}]
set_property PACKAGE_PIN B5 [get_ports {CAM0_YOUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[5]}]
set_property PACKAGE_PIN B2 [get_ports {CAM0_YOUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[6]}]
set_property PACKAGE_PIN C2 [get_ports {CAM0_YOUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_YOUT[7]}]
set_property PACKAGE_PIN E1 [get_ports {CAM0_COUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[0]}]
set_property PACKAGE_PIN F1 [get_ports {CAM0_COUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[1]}]
set_property PACKAGE_PIN F2 [get_ports {CAM0_COUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[2]}]
set_property PACKAGE_PIN F3 [get_ports {CAM0_COUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[3]}]
set_property PACKAGE_PIN E3 [get_ports {CAM0_COUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[4]}]
set_property PACKAGE_PIN F4 [get_ports {CAM0_COUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[5]}]
set_property PACKAGE_PIN E4 [get_ports {CAM0_COUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[6]}]
set_property PACKAGE_PIN E5 [get_ports {CAM0_COUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM0_COUT[7]}]
create_clock -period 10.000 -name CAM0_PCLK [get_ports CAM0_PCLK]

# --- CAM1 ---
set_property PACKAGE_PIN C10 [get_ports CAM1_PCLK]
set_property IOSTANDARD LVCMOS33 [get_ports CAM1_PCLK]
set_property PACKAGE_PIN A11 [get_ports {CAM1_YOUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[0]}]
set_property PACKAGE_PIN B11 [get_ports {CAM1_YOUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[1]}]
set_property PACKAGE_PIN A9 [get_ports {CAM1_YOUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[2]}]
set_property PACKAGE_PIN B10 [get_ports {CAM1_YOUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[3]}]
set_property PACKAGE_PIN A8 [get_ports {CAM1_YOUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[4]}]
set_property PACKAGE_PIN B9 [get_ports {CAM1_YOUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[5]}]
set_property PACKAGE_PIN A7 [get_ports {CAM1_YOUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[6]}]
set_property PACKAGE_PIN B7 [get_ports {CAM1_YOUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_YOUT[7]}]
set_property PACKAGE_PIN E9 [get_ports {CAM1_COUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[0]}]
set_property PACKAGE_PIN E10 [get_ports {CAM1_COUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[1]}]
set_property PACKAGE_PIN E8 [get_ports {CAM1_COUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[2]}]
set_property PACKAGE_PIN F9 [get_ports {CAM1_COUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[3]}]
set_property PACKAGE_PIN F7 [get_ports {CAM1_COUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[4]}]
set_property PACKAGE_PIN F8 [get_ports {CAM1_COUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[5]}]
set_property PACKAGE_PIN E6 [get_ports {CAM1_COUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[6]}]
set_property PACKAGE_PIN F6 [get_ports {CAM1_COUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM1_COUT[7]}]
set_property PACKAGE_PIN C9 [get_ports TRIG_IN1]
set_property IOSTANDARD LVCMOS33 [get_ports TRIG_IN1]
create_clock -period 10.000 -name CAM1_PCLK [get_ports CAM1_PCLK]

# --- CAM2 ---
set_property PACKAGE_PIN M12 [get_ports CAM2_PCLK]
set_property IOSTANDARD LVCMOS33 [get_ports CAM2_PCLK]
set_property PACKAGE_PIN K10 [get_ports {CAM2_YOUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[0]}]
set_property PACKAGE_PIN K11 [get_ports {CAM2_YOUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[1]}]
set_property PACKAGE_PIN L11 [get_ports {CAM2_YOUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[2]}]
set_property PACKAGE_PIN L12 [get_ports {CAM2_YOUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[3]}]
set_property PACKAGE_PIN K13 [get_ports {CAM2_YOUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[4]}]
set_property PACKAGE_PIN L13 [get_ports {CAM2_YOUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[5]}]
set_property PACKAGE_PIN M10 [get_ports {CAM2_YOUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[6]}]
set_property PACKAGE_PIN N10 [get_ports {CAM2_YOUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_YOUT[7]}]
set_property PACKAGE_PIN P10 [get_ports {CAM2_COUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[0]}]
set_property PACKAGE_PIN P11 [get_ports {CAM2_COUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[1]}]
set_property PACKAGE_PIN P12 [get_ports {CAM2_COUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[2]}]
set_property PACKAGE_PIN P13 [get_ports {CAM2_COUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[3]}]
set_property PACKAGE_PIN R13 [get_ports {CAM2_COUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[4]}]
set_property PACKAGE_PIN R14 [get_ports {CAM2_COUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[5]}]
set_property PACKAGE_PIN P15 [get_ports {CAM2_COUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[6]}]
set_property PACKAGE_PIN R15 [get_ports {CAM2_COUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM2_COUT[7]}]
set_property PACKAGE_PIN M11 [get_ports TRIG_IN2]
set_property IOSTANDARD LVCMOS33 [get_ports TRIG_IN2]
create_clock -period 13.468 -name CAM2_PCLK [get_ports CAM2_PCLK]

# --- CAM3 ---
set_property PACKAGE_PIN E13 [get_ports CAM3_PCLK]
set_property IOSTANDARD LVCMOS33 [get_ports CAM3_PCLK]
set_property PACKAGE_PIN A14 [get_ports {CAM3_YOUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[0]}]
set_property PACKAGE_PIN B14 [get_ports {CAM3_YOUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[1]}]
set_property PACKAGE_PIN A12 [get_ports {CAM3_YOUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[2]}]
set_property PACKAGE_PIN A13 [get_ports {CAM3_YOUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[3]}]
set_property PACKAGE_PIN B12 [get_ports {CAM3_YOUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[4]}]
set_property PACKAGE_PIN C13 [get_ports {CAM3_YOUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[5]}]
set_property PACKAGE_PIN C12 [get_ports {CAM3_YOUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[6]}]
set_property PACKAGE_PIN D12 [get_ports {CAM3_YOUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_YOUT[7]}]
set_property PACKAGE_PIN G10 [get_ports {CAM3_COUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[0]}]
set_property PACKAGE_PIN H10 [get_ports {CAM3_COUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[1]}]
set_property PACKAGE_PIN H12 [get_ports {CAM3_COUT[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[2]}]
set_property PACKAGE_PIN H13 [get_ports {CAM3_COUT[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[3]}]
set_property PACKAGE_PIN J10 [get_ports {CAM3_COUT[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[4]}]
set_property PACKAGE_PIN J11 [get_ports {CAM3_COUT[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[5]}]
set_property PACKAGE_PIN J12 [get_ports {CAM3_COUT[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[6]}]
set_property PACKAGE_PIN J13 [get_ports {CAM3_COUT[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {CAM3_COUT[7]}]
set_property PACKAGE_PIN D13 [get_ports TRIG_IN3]
set_property IOSTANDARD LVCMOS33 [get_ports TRIG_IN3]
create_clock -period 13.468 -name CAM3_PCLK [get_ports CAM3_PCLK]

# --- CAM4 ---
set_property PACKAGE_PIN E28 [get_ports CAM4_PCLK]
set_property IOSTANDARD LVCMOS18 [get_ports CAM4_PCLK]
set_property PACKAGE_PIN A29 [get_ports {CAM4_YOUT[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[0]}]
set_property PACKAGE_PIN A28 [get_ports {CAM4_YOUT[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[1]}]
set_property PACKAGE_PIN A26 [get_ports {CAM4_YOUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[2]}]
set_property PACKAGE_PIN B26 [get_ports {CAM4_YOUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[3]}]
set_property PACKAGE_PIN A27 [get_ports {CAM4_YOUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[4]}]
set_property PACKAGE_PIN B27 [get_ports {CAM4_YOUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[5]}]
set_property PACKAGE_PIN A31 [get_ports {CAM4_YOUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[6]}]
set_property PACKAGE_PIN B31 [get_ports {CAM4_YOUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_YOUT[7]}]
set_property PACKAGE_PIN C28 [get_ports {CAM4_COUT[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[0]}]
set_property PACKAGE_PIN C27 [get_ports {CAM4_COUT[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[1]}]
set_property PACKAGE_PIN B29 [get_ports {CAM4_COUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[2]}]
set_property PACKAGE_PIN C29 [get_ports {CAM4_COUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[3]}]
set_property PACKAGE_PIN B30 [get_ports {CAM4_COUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[4]}]
set_property PACKAGE_PIN C30 [get_ports {CAM4_COUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[5]}]
set_property PACKAGE_PIN D28 [get_ports {CAM4_COUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[6]}]
set_property PACKAGE_PIN D27 [get_ports {CAM4_COUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM4_COUT[7]}]
set_property PACKAGE_PIN F29 [get_ports TRIG_IN4]
set_property IOSTANDARD LVCMOS18 [get_ports TRIG_IN4]
create_clock -period 13.468 -name CAM4_PCLK [get_ports CAM4_PCLK]

# --- CAM5 ---
set_property PACKAGE_PIN G27 [get_ports CAM5_PCLK]
set_property IOSTANDARD LVCMOS18 [get_ports CAM5_PCLK]
set_property PACKAGE_PIN J31 [get_ports {CAM5_YOUT[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[0]}]
set_property PACKAGE_PIN J30 [get_ports {CAM5_YOUT[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[1]}]
set_property PACKAGE_PIN F26 [get_ports {CAM5_YOUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[2]}]
set_property PACKAGE_PIN K25 [get_ports {CAM5_YOUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[3]}]
set_property PACKAGE_PIN G26 [get_ports {CAM5_YOUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[4]}]
set_property PACKAGE_PIN G25 [get_ports {CAM5_YOUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[5]}]
set_property PACKAGE_PIN J26 [get_ports {CAM5_YOUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[6]}]
set_property PACKAGE_PIN K26 [get_ports {CAM5_YOUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_YOUT[7]}]
set_property PACKAGE_PIN H25 [get_ports {CAM5_COUT[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[0]}]
set_property PACKAGE_PIN J25 [get_ports {CAM5_COUT[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[1]}]
set_property PACKAGE_PIN L27 [get_ports {CAM5_COUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[2]}]
set_property PACKAGE_PIN L26 [get_ports {CAM5_COUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[3]}]
set_property PACKAGE_PIN H27 [get_ports {CAM5_COUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[4]}]
set_property PACKAGE_PIN J27 [get_ports {CAM5_COUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[5]}]
set_property PACKAGE_PIN M26 [get_ports {CAM5_COUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[6]}]
set_property PACKAGE_PIN M25 [get_ports {CAM5_COUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {CAM5_COUT[7]}]
set_property PACKAGE_PIN F31 [get_ports TRIG_IN5]
set_property IOSTANDARD LVCMOS18 [get_ports TRIG_IN5]
create_clock -period 13.468 -name CAM5_PCLK [get_ports CAM5_PCLK]


#####################################################################
# IEG0 / IEG1 outputs (same as 5cam)
#####################################################################

# Re-mapped to avoid SDA/SCL(E19/E18) conflict (see note at top)
set_property PACKAGE_PIN N15 [get_ports {IEG0_DOUT[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {IEG0_DOUT[0]}]
set_property PACKAGE_PIN M14 [get_ports {IEG0_DOUT[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {IEG0_DOUT[1]}]

set_property PACKAGE_PIN C4 [get_ports STROBE_OUT0]
set_property IOSTANDARD LVCMOS33 [get_ports STROBE_OUT0]
set_property PACKAGE_PIN H15 [get_ports IEG0_HSYNC]
set_property IOSTANDARD LVCMOS18 [get_ports IEG0_HSYNC]
set_property PACKAGE_PIN J15 [get_ports IEG0_VSYNC]
set_property IOSTANDARD LVCMOS18 [get_ports IEG0_VSYNC]
set_property PACKAGE_PIN G19 [get_ports IEG0_PCLK]
set_property IOSTANDARD LVCMOS18 [get_ports IEG0_PCLK]
set_property PACKAGE_PIN F16 [get_ports {IEG0_DOUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[2]}]
set_property PACKAGE_PIN H19 [get_ports {IEG0_DOUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[3]}]
set_property PACKAGE_PIN G16 [get_ports {IEG0_DOUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[4]}]
set_property PACKAGE_PIN G17 [get_ports {IEG0_DOUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[5]}]
set_property PACKAGE_PIN G15 [get_ports {IEG0_DOUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[6]}]
set_property PACKAGE_PIN G14 [get_ports {IEG0_DOUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[7]}]
set_property PACKAGE_PIN H14 [get_ports {IEG0_DOUT[8]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[8]}]
set_property PACKAGE_PIN H17 [get_ports {IEG0_DOUT[9]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[9]}]
set_property PACKAGE_PIN H18 [get_ports {IEG0_DOUT[10]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[10]}]
set_property PACKAGE_PIN J18 [get_ports {IEG0_DOUT[11]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[11]}]
set_property PACKAGE_PIN J17 [get_ports {IEG0_DOUT[12]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[12]}]
set_property PACKAGE_PIN K14 [get_ports {IEG0_DOUT[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[13]}]
set_property PACKAGE_PIN K15 [get_ports {IEG0_DOUT[14]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[14]}]
set_property PACKAGE_PIN J16 [get_ports {IEG0_DOUT[15]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[15]}]
set_property PACKAGE_PIN L16 [get_ports {IEG0_DOUT[16]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[16]}]
set_property PACKAGE_PIN L17 [get_ports {IEG0_DOUT[17]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[17]}]
set_property PACKAGE_PIN K18 [get_ports {IEG0_DOUT[18]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[18]}]
set_property PACKAGE_PIN L18 [get_ports {IEG0_DOUT[19]}]
set_property IOSTANDARD LVCMOS18 [get_ports {IEG0_DOUT[19]}]

#set_property PACKAGE_PIN AL20 [get_ports IEG1_HSYNC]
#set_property IOSTANDARD LVCMOS18 [get_ports IEG1_HSYNC]
#set_property PACKAGE_PIN AM20 [get_ports IEG1_VSYNC]
#set_property IOSTANDARD LVCMOS18 [get_ports IEG1_VSYNC]
#set_property PACKAGE_PIN AP18 [get_ports IEG1_PCLK]
#set_property IOSTANDARD LVCMOS18 [get_ports IEG1_PCLK]
#set_property PACKAGE_PIN AU18 [get_ports {IEG1_DOUT[0]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[0]}]
#set_property PACKAGE_PIN AU19 [get_ports {IEG1_DOUT[1]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[1]}]
#set_property PACKAGE_PIN AU17 [get_ports {IEG1_DOUT[2]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[2]}]
#set_property PACKAGE_PIN AT19 [get_ports {IEG1_DOUT[3]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[3]}]
#set_property PACKAGE_PIN AT20 [get_ports {IEG1_DOUT[4]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[4]}]
#set_property PACKAGE_PIN AP20 [get_ports {IEG1_DOUT[5]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[5]}]
#set_property PACKAGE_PIN AP21 [get_ports {IEG1_DOUT[6]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[6]}]
#set_property PACKAGE_PIN AM22 [get_ports {IEG1_DOUT[7]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[7]}]
#set_property PACKAGE_PIN AL22 [get_ports {IEG1_DOUT[8]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[8]}]
#set_property PACKAGE_PIN D21 [get_ports {IEG1_DOUT[9]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[9]}]
#set_property PACKAGE_PIN E21 [get_ports {IEG1_DOUT[10]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[10]}]
#set_property PACKAGE_PIN D23 [get_ports {IEG1_DOUT[11]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[11]}]
#set_property PACKAGE_PIN D25 [get_ports {IEG1_DOUT[12]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[12]}]
#set_property PACKAGE_PIN E25 [get_ports {IEG1_DOUT[13]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[13]}]
#set_property PACKAGE_PIN E24 [get_ports {IEG1_DOUT[14]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[14]}]
#set_property PACKAGE_PIN F22 [get_ports {IEG1_DOUT[15]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[15]}]
#set_property PACKAGE_PIN G22 [get_ports {IEG1_DOUT[16]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[16]}]
#set_property PACKAGE_PIN G24 [get_ports {IEG1_DOUT[17]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[17]}]
#set_property PACKAGE_PIN H24 [get_ports {IEG1_DOUT[18]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[18]}]
#set_property PACKAGE_PIN J21 [get_ports {IEG1_DOUT[19]}]
#set_property IOSTANDARD LVCMOS18 [get_ports {IEG1_DOUT[19]}]

# 6개의 카메라 클럭은 서로 비동기(Asynchronous)이거나 물리적으로 배타적(Physically Exclusive)임
set_clock_groups -physically_exclusive -group [get_clocks CAM0_PCLK] -group [get_clocks CAM1_PCLK] -group [get_clocks CAM2_PCLK] -group [get_clocks CAM3_PCLK] -group [get_clocks CAM4_PCLK] -group [get_clocks CAM5_PCLK]


##############################################################

#SD-HDI (bank 71)
#NET "HD_DE"		LOC = F14
#NET "HD_VSYNC"	LOC = F18
#NET "HD_HSYNC"	LOC = F19
#NET "HD_PCLK"		LOC = F17

#NET "HD_DOUT[0]"	LOC = E16
#NET "HD_DOUT[1]"	LOC = E15
#NET "HD_DOUT[2]"	LOC = D18
#NET "HD_DOUT[3]"	LOC = D17
#NET "HD_DOUT[4]"	LOC = D16
#NET "HD_DOUT[5]"	LOC = D15
#NET "HD_DOUT[6]"	LOC = E14
#NET "HD_DOUT[7]"	LOC = C19
#NET "HD_DOUT[8]"	LOC = C18
#NET "HD_DOUT[9]"	LOC = C17
#NET "HD_DOUT[10]"	LOC = C15
#NET "HD_DOUT[11]"	LOC = C14
#NET "HD_DOUT[12]"	LOC = B19
#NET "HD_DOUT[13]"	LOC = A19
#NET "HD_DOUT[14]"	LOC = B16
#NET "HD_DOUT[15]"	LOC = B15
#NET "HD_DOUT[16]"	LOC = A18
#NET "HD_DOUT[17]"	LOC = A17
#NET "HD_DOUT[18]"	LOC = B17
#NET "HD_DOUT[19]"	LOC = A16
set_property PACKAGE_PIN F14 [get_ports HD_DE]
set_property IOSTANDARD LVCMOS18 [get_ports HD_DE]
set_property PACKAGE_PIN F18 [get_ports HD_VSYNC]
set_property IOSTANDARD LVCMOS18 [get_ports HD_VSYNC]
set_property PACKAGE_PIN F19 [get_ports HD_HSYNC]
set_property IOSTANDARD LVCMOS18 [get_ports HD_HSYNC]
set_property PACKAGE_PIN F17 [get_ports HD_PCLK]
set_property IOSTANDARD LVCMOS18 [get_ports HD_PCLK]
set_property PACKAGE_PIN E16 [get_ports {HD_DOUT[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[0]}]
set_property PACKAGE_PIN E15 [get_ports {HD_DOUT[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[1]}]
set_property PACKAGE_PIN D18 [get_ports {HD_DOUT[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[2]}]
set_property PACKAGE_PIN D17 [get_ports {HD_DOUT[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[3]}]
set_property PACKAGE_PIN D16 [get_ports {HD_DOUT[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[4]}]
set_property PACKAGE_PIN D15 [get_ports {HD_DOUT[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[5]}]
set_property PACKAGE_PIN E14 [get_ports {HD_DOUT[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[6]}]
set_property PACKAGE_PIN C19 [get_ports {HD_DOUT[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[7]}]
set_property PACKAGE_PIN C18 [get_ports {HD_DOUT[8]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[8]}]
set_property PACKAGE_PIN C17 [get_ports {HD_DOUT[9]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[9]}]
set_property PACKAGE_PIN C15 [get_ports {HD_DOUT[10]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[10]}]
set_property PACKAGE_PIN C14 [get_ports {HD_DOUT[11]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[11]}]
set_property PACKAGE_PIN B19 [get_ports {HD_DOUT[12]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[12]}]
set_property PACKAGE_PIN A19 [get_ports {HD_DOUT[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[13]}]
set_property PACKAGE_PIN B16 [get_ports {HD_DOUT[14]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[14]}]
set_property PACKAGE_PIN B15 [get_ports {HD_DOUT[15]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[15]}]
set_property PACKAGE_PIN A18 [get_ports {HD_DOUT[16]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[16]}]
set_property PACKAGE_PIN A17 [get_ports {HD_DOUT[17]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[17]}]
set_property PACKAGE_PIN B17 [get_ports {HD_DOUT[18]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[18]}]
set_property PACKAGE_PIN A16 [get_ports {HD_DOUT[19]}]
set_property IOSTANDARD LVCMOS18 [get_ports {HD_DOUT[19]}]
##############################################################

# 기존 4에서 8로 변경
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 2.7 [current_design]
# 압축 옵션은 끄셔도 됩니다 (용량이 충분해지므로)
# set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.PERSIST NO [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
