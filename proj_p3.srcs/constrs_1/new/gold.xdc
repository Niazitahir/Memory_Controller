
set_property IOSTANDARD LVCMOS33 [get_ports resetn]

set_property PULLUP true [get_ports resetn]
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L12P_T1_MRCC_35 Sch=clk100mhz