// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
// Date        : Tue Mar 10 22:45:20 2026
// Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_1_stub.v
// Design      : design_1_mem_controller_0_1
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "mem_controller,Vivado 2018.3.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(aclk, aresetn, a_en, a_we, a_addr, a_din, a_dout, b_en, 
  b_we, b_addr, b_din, b_dout, bram_ena, bram_wea, bram_addra, bram_dina, bram_douta, bram_rsta, 
  bram_enb, bram_web, bram_addrb, bram_dinb, bram_doutb, bram_rstb, bram_en, bram_addr, bram_din, 
  bram_we, bram_dout, S_AXI_AWADDR, S_AXI_AWVALID, S_AXI_AWREADY, S_AXI_WDATA, S_AXI_WSTRB, 
  S_AXI_WVALID, S_AXI_WREADY, S_AXI_BRESP, S_AXI_BVALID, S_AXI_BREADY, S_AXI_ARADDR, 
  S_AXI_ARVALID, S_AXI_ARREADY, S_AXI_RDATA, S_AXI_RRESP, S_AXI_RVALID, S_AXI_RREADY, 
  M_AXI_AWADDR, M_AXI_AWVALID, M_AXI_AWREADY, M_AXI_WDATA, M_AXI_WSTRB, M_AXI_WVALID, 
  M_AXI_WREADY, M_AXI_BRESP, M_AXI_BVALID, M_AXI_BREADY)
/* synthesis syn_black_box black_box_pad_pin="aclk,aresetn,a_en,a_we,a_addr[3:0],a_din[31:0],a_dout[31:0],b_en,b_we,b_addr[3:0],b_din[31:0],b_dout[31:0],bram_ena,bram_wea[3:0],bram_addra[3:0],bram_dina[31:0],bram_douta[31:0],bram_rsta,bram_enb,bram_web[3:0],bram_addrb[3:0],bram_dinb[31:0],bram_doutb[31:0],bram_rstb,bram_en,bram_addr[31:0],bram_din[31:0],bram_we[3:0],bram_dout[31:0],S_AXI_AWADDR[31:0],S_AXI_AWVALID,S_AXI_AWREADY,S_AXI_WDATA[31:0],S_AXI_WSTRB[3:0],S_AXI_WVALID,S_AXI_WREADY,S_AXI_BRESP[1:0],S_AXI_BVALID,S_AXI_BREADY,S_AXI_ARADDR[31:0],S_AXI_ARVALID,S_AXI_ARREADY,S_AXI_RDATA[31:0],S_AXI_RRESP[1:0],S_AXI_RVALID,S_AXI_RREADY,M_AXI_AWADDR[31:0],M_AXI_AWVALID,M_AXI_AWREADY,M_AXI_WDATA[31:0],M_AXI_WSTRB[3:0],M_AXI_WVALID,M_AXI_WREADY,M_AXI_BRESP[1:0],M_AXI_BVALID,M_AXI_BREADY" */;
  input aclk;
  input aresetn;
  input a_en;
  input a_we;
  input [3:0]a_addr;
  input [31:0]a_din;
  output [31:0]a_dout;
  input b_en;
  input b_we;
  input [3:0]b_addr;
  input [31:0]b_din;
  output [31:0]b_dout;
  output bram_ena;
  output [3:0]bram_wea;
  output [3:0]bram_addra;
  output [31:0]bram_dina;
  input [31:0]bram_douta;
  output bram_rsta;
  output bram_enb;
  output [3:0]bram_web;
  output [3:0]bram_addrb;
  output [31:0]bram_dinb;
  input [31:0]bram_doutb;
  output bram_rstb;
  input bram_en;
  input [31:0]bram_addr;
  input [31:0]bram_din;
  input [3:0]bram_we;
  output [31:0]bram_dout;
  input [31:0]S_AXI_AWADDR;
  input S_AXI_AWVALID;
  output S_AXI_AWREADY;
  input [31:0]S_AXI_WDATA;
  input [3:0]S_AXI_WSTRB;
  input S_AXI_WVALID;
  output S_AXI_WREADY;
  output [1:0]S_AXI_BRESP;
  output S_AXI_BVALID;
  input S_AXI_BREADY;
  input [31:0]S_AXI_ARADDR;
  input S_AXI_ARVALID;
  output S_AXI_ARREADY;
  output [31:0]S_AXI_RDATA;
  output [1:0]S_AXI_RRESP;
  output S_AXI_RVALID;
  input S_AXI_RREADY;
  output [31:0]M_AXI_AWADDR;
  output M_AXI_AWVALID;
  input M_AXI_AWREADY;
  output [31:0]M_AXI_WDATA;
  output [3:0]M_AXI_WSTRB;
  output M_AXI_WVALID;
  input M_AXI_WREADY;
  input [1:0]M_AXI_BRESP;
  input M_AXI_BVALID;
  output M_AXI_BREADY;
endmodule
