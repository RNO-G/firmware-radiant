# Don't change this. The actual clock frequency isn't this high, we just make it this high to get safe timing.
create_clock -period 20.000 -name clk50_clock -waveform {0.000 10.000} [get_ports CLK50]
create_clock -period 5.000 -name clk200_clock -waveform {0.000 2.500} [get_ports SYS_CLK_P]

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
