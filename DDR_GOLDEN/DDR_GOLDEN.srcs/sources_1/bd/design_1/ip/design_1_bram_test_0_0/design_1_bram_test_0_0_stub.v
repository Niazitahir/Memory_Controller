// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
// Date        : Tue Mar 10 14:04:57 2026
// Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               C:/Users/khanm/AppData/Roaming/Xilinx/Vivado/proj/DDR_GOLDEN/DDR_GOLDEN.srcs/sources_1/bd/design_1/ip/design_1_bram_test_0_0/design_1_bram_test_0_0_stub.v
// Design      : design_1_bram_test_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "bram_test,Vivado 2018.3.1" *)
module design_1_bram_test_0_0(clk, reset_n, bram_en, bram_addr, bram_din, 
  bram_we, bram_dout, bram_ready, tester, test_pass)
/* synthesis syn_black_box black_box_pad_pin="clk,reset_n,bram_en,bram_addr[31:0],bram_din[31:0],bram_we[3:0],bram_dout[31:0],bram_ready,tester,test_pass" */;
  input clk;
  input reset_n;
  output bram_en;
  output [31:0]bram_addr;
  output [31:0]bram_din;
  output [3:0]bram_we;
  input [31:0]bram_dout;
  input bram_ready;
  input tester;
  output test_pass;
endmodule
