# Don't change this. The actual clock frequency isn't this high, we just make it this high to get safe timing.
create_clock -period 20.000 -name clk50_clock -waveform {0.000 10.000} [get_ports CLK50]
create_clock -period 5.000 -name clk200_clock -waveform {0.000 2.500} [get_ports SYS_CLK_P]

set_max_delay -datapath_only -from clk50_clock -to clk200_clock 20.000
set_max_delay -datapath_only -from clk200_clock -to clk50_clock 20.000

set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVDS_25} [get_ports PULSE_P]
set_property -dict {PACKAGE_PIN A19 IOSTANDARD LVDS_25} [get_ports PULSE_N]

set_property -dict {PACKAGE_PIN P4 IOSTANDARD LVDS_25} [get_ports SYS_CLK_P]
set_property -dict {PACKAGE_PIN N4 IOSTANDARD LVDS_25} [get_ports SYS_CLK_N]
set_property -dict {PACKAGE_PIN H15 IOSTANDARD LVCMOS25} [get_ports F_LED]
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS25} [get_ports CLK50_EN]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS25} [get_ports CLK50]
set_property -dict {PACKAGE_PIN K7 IOSTANDARD LVCMOS25} [get_ports BM_TX]
set_property -dict {PACKAGE_PIN K6 IOSTANDARD LVCMOS25} [get_ports BM_RX]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# CDAT_TDI right = G19 left = V22
# CCLK_TMS right = E18 left = U22
# SCLK_TCK right = F20 left = AA22
# SSINCR_TDO right = D19 left = AA23
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS25} [get_ports {CDAT_TDI[1]}]
set_property -dict {PACKAGE_PIN V22 IOSTANDARD LVCMOS25} [get_ports {CDAT_TDI[0]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS25} [get_ports {CCLK_TMS[1]}]
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS25} [get_ports {CCLK_TMS[0]}]
set_property -dict {PACKAGE_PIN F20 IOSTANDARD LVCMOS25} [get_ports {SCLK_TCK[1]}]
set_property -dict {PACKAGE_PIN AA22 IOSTANDARD LVCMOS25} [get_ports {SCLK_TCK[0]}]
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS25 PULLDOWN TRUE} [get_ports {SSINCR_TDO[1]}]
set_property -dict {PACKAGE_PIN AA23 IOSTANDARD LVCMOS25 PULLDOWN TRUE} [get_ports {SSINCR_TDO[0]}]

set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS25} [get_ports JTAGENB]

set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS25} [get_ports CS_B]
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS25} [get_ports MOSI]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS25} [get_ports MISO]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS25} [get_ports WPB]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS25} [get_ports HOLDB]

set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_VOLTAGE 2.5 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets CLK50_IBUF_BUFG]
