// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
// Date        : Mon Mar  2 23:16:11 2026
// Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_0_sim_netlist.v
// Design      : design_1_mem_controller_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master
   (M_AXI_AWVALID,
    M_AXI_WVALID,
    M_AXI_BREADY,
    aclk,
    M_AXI_BREADY_reg_0,
    M_AXI_WREADY,
    M_AXI_AWREADY,
    M_AXI_BVALID);
  output M_AXI_AWVALID;
  output M_AXI_WVALID;
  output M_AXI_BREADY;
  input aclk;
  input M_AXI_BREADY_reg_0;
  input M_AXI_WREADY;
  input M_AXI_AWREADY;
  input M_AXI_BVALID;

  wire \FSM_onehot_state[0]_i_1_n_0 ;
  wire \FSM_onehot_state[1]_i_1_n_0 ;
  wire \FSM_onehot_state[2]_i_1_n_0 ;
  wire \FSM_onehot_state_reg_n_0_[0] ;
  wire \FSM_onehot_state_reg_n_0_[1] ;
  wire \FSM_onehot_state_reg_n_0_[2] ;
  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_AWVALID_i_1_n_0;
  wire M_AXI_BREADY;
  wire M_AXI_BREADY_i_1_n_0;
  wire M_AXI_BREADY_i_2_n_0;
  wire M_AXI_BREADY_reg_0;
  wire M_AXI_BVALID;
  wire M_AXI_WREADY;
  wire M_AXI_WVALID;
  wire M_AXI_WVALID_i_1_n_0;
  wire aclk;

  LUT4 #(
    .INIT(16'hDDC8)) 
    \FSM_onehot_state[0]_i_1 
       (.I0(M_AXI_BREADY_i_2_n_0),
        .I1(\FSM_onehot_state_reg_n_0_[2] ),
        .I2(M_AXI_BVALID),
        .I3(\FSM_onehot_state_reg_n_0_[0] ),
        .O(\FSM_onehot_state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'hFF15EA00)) 
    \FSM_onehot_state[1]_i_1 
       (.I0(M_AXI_BREADY_i_2_n_0),
        .I1(\FSM_onehot_state_reg_n_0_[2] ),
        .I2(M_AXI_BVALID),
        .I3(\FSM_onehot_state_reg_n_0_[0] ),
        .I4(\FSM_onehot_state_reg_n_0_[1] ),
        .O(\FSM_onehot_state[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'hEE04)) 
    \FSM_onehot_state[2]_i_1 
       (.I0(M_AXI_BREADY_i_2_n_0),
        .I1(\FSM_onehot_state_reg_n_0_[2] ),
        .I2(M_AXI_BVALID),
        .I3(\FSM_onehot_state_reg_n_0_[1] ),
        .O(\FSM_onehot_state[2]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "ST_AW_W:010,ST_B:100,ST_DONE:001," *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_state_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\FSM_onehot_state[0]_i_1_n_0 ),
        .PRE(M_AXI_BREADY_reg_0),
        .Q(\FSM_onehot_state_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "ST_AW_W:010,ST_B:100,ST_DONE:001," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_state_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(M_AXI_BREADY_reg_0),
        .D(\FSM_onehot_state[1]_i_1_n_0 ),
        .Q(\FSM_onehot_state_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "ST_AW_W:010,ST_B:100,ST_DONE:001," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_state_reg[2] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(M_AXI_BREADY_reg_0),
        .D(\FSM_onehot_state[2]_i_1_n_0 ),
        .Q(\FSM_onehot_state_reg_n_0_[2] ));
  LUT3 #(
    .INIT(8'h7A)) 
    M_AXI_AWVALID_i_1
       (.I0(M_AXI_AWVALID),
        .I1(M_AXI_AWREADY),
        .I2(\FSM_onehot_state_reg_n_0_[1] ),
        .O(M_AXI_AWVALID_i_1_n_0));
  FDCE M_AXI_AWVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(M_AXI_BREADY_reg_0),
        .D(M_AXI_AWVALID_i_1_n_0),
        .Q(M_AXI_AWVALID));
  LUT4 #(
    .INIT(16'hBFAA)) 
    M_AXI_BREADY_i_1
       (.I0(M_AXI_BREADY_i_2_n_0),
        .I1(\FSM_onehot_state_reg_n_0_[2] ),
        .I2(M_AXI_BVALID),
        .I3(M_AXI_BREADY),
        .O(M_AXI_BREADY_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h80000000)) 
    M_AXI_BREADY_i_2
       (.I0(M_AXI_WVALID),
        .I1(M_AXI_WREADY),
        .I2(\FSM_onehot_state_reg_n_0_[1] ),
        .I3(M_AXI_AWREADY),
        .I4(M_AXI_AWVALID),
        .O(M_AXI_BREADY_i_2_n_0));
  FDCE M_AXI_BREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(M_AXI_BREADY_reg_0),
        .D(M_AXI_BREADY_i_1_n_0),
        .Q(M_AXI_BREADY));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'h7A)) 
    M_AXI_WVALID_i_1
       (.I0(M_AXI_WVALID),
        .I1(M_AXI_WREADY),
        .I2(\FSM_onehot_state_reg_n_0_[1] ),
        .O(M_AXI_WVALID_i_1_n_0));
  FDCE M_AXI_WVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(M_AXI_BREADY_reg_0),
        .D(M_AXI_WVALID_i_1_n_0),
        .Q(M_AXI_WVALID));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave
   (S_AXI_WREADY,
    aresetn_0,
    S_AXI_AWREADY,
    S_AXI_ARREADY,
    S_AXI_RVALID,
    S_AXI_BVALID_reg_0,
    aclk,
    S_AXI_WVALID,
    S_AXI_AWVALID,
    S_AXI_RREADY,
    S_AXI_ARVALID,
    S_AXI_BREADY,
    aresetn);
  output S_AXI_WREADY;
  output aresetn_0;
  output S_AXI_AWREADY;
  output S_AXI_ARREADY;
  output S_AXI_RVALID;
  output S_AXI_BVALID_reg_0;
  input aclk;
  input S_AXI_WVALID;
  input S_AXI_AWVALID;
  input S_AXI_RREADY;
  input S_AXI_ARVALID;
  input S_AXI_BREADY;
  input aresetn;

  wire S_AXI_ARREADY;
  wire S_AXI_ARREADY0;
  wire S_AXI_ARVALID;
  wire S_AXI_AWREADY;
  wire S_AXI_AWREADY0;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID_i_1_n_0;
  wire S_AXI_BVALID_reg_0;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID;
  wire S_AXI_RVALID_i_1_n_0;
  wire S_AXI_WREADY;
  wire S_AXI_WREADY0;
  wire S_AXI_WVALID;
  wire aclk;
  wire ar_hs_d1;
  wire ar_hs_d10;
  wire aresetn;
  wire aresetn_0;
  wire aw_hs_done_i_1_n_0;
  wire aw_hs_done_reg_n_0;
  wire w_hs_done_i_1_n_0;
  wire w_hs_done_reg_n_0;

  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'h2)) 
    S_AXI_ARREADY_i_1
       (.I0(S_AXI_ARVALID),
        .I1(S_AXI_ARREADY),
        .O(S_AXI_ARREADY0));
  FDCE S_AXI_ARREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_ARREADY0),
        .Q(S_AXI_ARREADY));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h2)) 
    S_AXI_AWREADY_i_1
       (.I0(S_AXI_AWVALID),
        .I1(S_AXI_AWREADY),
        .O(S_AXI_AWREADY0));
  LUT1 #(
    .INIT(2'h1)) 
    S_AXI_AWREADY_i_2
       (.I0(aresetn),
        .O(aresetn_0));
  FDCE S_AXI_AWREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_AWREADY0),
        .Q(S_AXI_AWREADY));
  LUT4 #(
    .INIT(16'h7222)) 
    S_AXI_BVALID_i_1
       (.I0(S_AXI_BVALID_reg_0),
        .I1(S_AXI_BREADY),
        .I2(w_hs_done_reg_n_0),
        .I3(aw_hs_done_reg_n_0),
        .O(S_AXI_BVALID_i_1_n_0));
  FDCE S_AXI_BVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_BVALID_i_1_n_0),
        .Q(S_AXI_BVALID_reg_0));
  LUT3 #(
    .INIT(8'h3A)) 
    S_AXI_RVALID_i_1
       (.I0(ar_hs_d1),
        .I1(S_AXI_RREADY),
        .I2(S_AXI_RVALID),
        .O(S_AXI_RVALID_i_1_n_0));
  FDCE S_AXI_RVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_RVALID_i_1_n_0),
        .Q(S_AXI_RVALID));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'h2)) 
    S_AXI_WREADY_i_1
       (.I0(S_AXI_WVALID),
        .I1(S_AXI_WREADY),
        .O(S_AXI_WREADY0));
  FDCE S_AXI_WREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_WREADY0),
        .Q(S_AXI_WREADY));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'h8)) 
    ar_hs_d1_i_1
       (.I0(S_AXI_ARREADY),
        .I1(S_AXI_ARVALID),
        .O(ar_hs_d10));
  FDCE ar_hs_d1_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(ar_hs_d10),
        .Q(ar_hs_d1));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT5 #(
    .INIT(32'h77777000)) 
    aw_hs_done_i_1
       (.I0(S_AXI_BVALID_reg_0),
        .I1(S_AXI_BREADY),
        .I2(S_AXI_AWVALID),
        .I3(S_AXI_AWREADY),
        .I4(aw_hs_done_reg_n_0),
        .O(aw_hs_done_i_1_n_0));
  FDCE aw_hs_done_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(aw_hs_done_i_1_n_0),
        .Q(aw_hs_done_reg_n_0));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h77777000)) 
    w_hs_done_i_1
       (.I0(S_AXI_BVALID_reg_0),
        .I1(S_AXI_BREADY),
        .I2(S_AXI_WVALID),
        .I3(S_AXI_WREADY),
        .I4(w_hs_done_reg_n_0),
        .O(w_hs_done_i_1_n_0));
  FDCE w_hs_done_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(w_hs_done_i_1_n_0),
        .Q(w_hs_done_reg_n_0));
endmodule

(* CHECK_LICENSE_TYPE = "design_1_mem_controller_0_0,mem_controller,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "package_project" *) 
(* X_CORE_INFO = "mem_controller,Vivado 2018.3.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (aclk,
    aresetn,
    S_AXI_AWADDR,
    S_AXI_AWVALID,
    S_AXI_AWREADY,
    S_AXI_WDATA,
    S_AXI_WSTRB,
    S_AXI_WVALID,
    S_AXI_WREADY,
    S_AXI_BRESP,
    S_AXI_BVALID,
    S_AXI_BREADY,
    S_AXI_ARADDR,
    S_AXI_ARVALID,
    S_AXI_ARREADY,
    S_AXI_RDATA,
    S_AXI_RRESP,
    S_AXI_RVALID,
    S_AXI_RREADY,
    M_AXI_AWADDR,
    M_AXI_AWVALID,
    M_AXI_AWREADY,
    M_AXI_WDATA,
    M_AXI_WSTRB,
    M_AXI_WVALID,
    M_AXI_WREADY,
    M_AXI_BRESP,
    M_AXI_BVALID,
    M_AXI_BREADY);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF M_AXI:S_AXI, ASSOCIATED_RESET aresetn, FREQ_HZ 81247969, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, INSERT_VIP 0" *) input aclk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input aresetn;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *) input [31:0]S_AXI_AWADDR;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input S_AXI_AWVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output S_AXI_AWREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input [31:0]S_AXI_WDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input [3:0]S_AXI_WSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input S_AXI_WVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output S_AXI_WREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output [1:0]S_AXI_BRESP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output S_AXI_BVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input S_AXI_BREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input [31:0]S_AXI_ARADDR;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input S_AXI_ARVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output S_AXI_ARREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output [31:0]S_AXI_RDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output [1:0]S_AXI_RRESP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output S_AXI_RVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 81247969, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *) input S_AXI_RREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWADDR" *) output [31:0]M_AXI_AWADDR;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWVALID" *) output M_AXI_AWVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWREADY" *) input M_AXI_AWREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WDATA" *) output [31:0]M_AXI_WDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WSTRB" *) output [3:0]M_AXI_WSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WVALID" *) output M_AXI_WVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WREADY" *) input M_AXI_WREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BRESP" *) input [1:0]M_AXI_BRESP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BVALID" *) input M_AXI_BVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BREADY" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 81247969, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *) output M_AXI_BREADY;

  wire \<const0> ;
  wire \<const1> ;
  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_BREADY;
  wire M_AXI_BVALID;
  wire M_AXI_WREADY;
  wire M_AXI_WVALID;
  wire S_AXI_ARREADY;
  wire S_AXI_ARVALID;
  wire S_AXI_AWREADY;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID;
  wire S_AXI_WREADY;
  wire S_AXI_WVALID;
  wire aclk;
  wire aresetn;

  assign M_AXI_AWADDR[31] = \<const0> ;
  assign M_AXI_AWADDR[30] = \<const0> ;
  assign M_AXI_AWADDR[29] = \<const0> ;
  assign M_AXI_AWADDR[28] = \<const0> ;
  assign M_AXI_AWADDR[27] = \<const0> ;
  assign M_AXI_AWADDR[26] = \<const0> ;
  assign M_AXI_AWADDR[25] = \<const0> ;
  assign M_AXI_AWADDR[24] = \<const0> ;
  assign M_AXI_AWADDR[23] = \<const0> ;
  assign M_AXI_AWADDR[22] = \<const0> ;
  assign M_AXI_AWADDR[21] = \<const0> ;
  assign M_AXI_AWADDR[20] = \<const0> ;
  assign M_AXI_AWADDR[19] = \<const0> ;
  assign M_AXI_AWADDR[18] = \<const0> ;
  assign M_AXI_AWADDR[17] = \<const0> ;
  assign M_AXI_AWADDR[16] = \<const0> ;
  assign M_AXI_AWADDR[15] = \<const0> ;
  assign M_AXI_AWADDR[14] = \<const0> ;
  assign M_AXI_AWADDR[13] = \<const0> ;
  assign M_AXI_AWADDR[12] = \<const0> ;
  assign M_AXI_AWADDR[11] = \<const0> ;
  assign M_AXI_AWADDR[10] = \<const0> ;
  assign M_AXI_AWADDR[9] = \<const0> ;
  assign M_AXI_AWADDR[8] = \<const0> ;
  assign M_AXI_AWADDR[7] = \<const0> ;
  assign M_AXI_AWADDR[6] = \<const0> ;
  assign M_AXI_AWADDR[5] = \<const0> ;
  assign M_AXI_AWADDR[4] = \<const0> ;
  assign M_AXI_AWADDR[3] = \<const0> ;
  assign M_AXI_AWADDR[2] = \<const0> ;
  assign M_AXI_AWADDR[1] = \<const0> ;
  assign M_AXI_AWADDR[0] = \<const0> ;
  assign M_AXI_WDATA[31] = \<const0> ;
  assign M_AXI_WDATA[30] = \<const0> ;
  assign M_AXI_WDATA[29] = \<const0> ;
  assign M_AXI_WDATA[28] = \<const0> ;
  assign M_AXI_WDATA[27] = \<const0> ;
  assign M_AXI_WDATA[26] = \<const0> ;
  assign M_AXI_WDATA[25] = \<const0> ;
  assign M_AXI_WDATA[24] = \<const0> ;
  assign M_AXI_WDATA[23] = \<const0> ;
  assign M_AXI_WDATA[22] = \<const0> ;
  assign M_AXI_WDATA[21] = \<const0> ;
  assign M_AXI_WDATA[20] = \<const0> ;
  assign M_AXI_WDATA[19] = \<const0> ;
  assign M_AXI_WDATA[18] = \<const0> ;
  assign M_AXI_WDATA[17] = \<const0> ;
  assign M_AXI_WDATA[16] = \<const0> ;
  assign M_AXI_WDATA[15] = \<const0> ;
  assign M_AXI_WDATA[14] = \<const0> ;
  assign M_AXI_WDATA[13] = \<const0> ;
  assign M_AXI_WDATA[12] = \<const0> ;
  assign M_AXI_WDATA[11] = \<const0> ;
  assign M_AXI_WDATA[10] = \<const0> ;
  assign M_AXI_WDATA[9] = \<const0> ;
  assign M_AXI_WDATA[8] = \<const0> ;
  assign M_AXI_WDATA[7] = \<const0> ;
  assign M_AXI_WDATA[6] = \<const0> ;
  assign M_AXI_WDATA[5] = \<const0> ;
  assign M_AXI_WDATA[4] = \<const0> ;
  assign M_AXI_WDATA[3] = \<const0> ;
  assign M_AXI_WDATA[2] = \<const0> ;
  assign M_AXI_WDATA[1] = \<const0> ;
  assign M_AXI_WDATA[0] = \<const0> ;
  assign M_AXI_WSTRB[3] = \<const1> ;
  assign M_AXI_WSTRB[2] = \<const1> ;
  assign M_AXI_WSTRB[1] = \<const1> ;
  assign M_AXI_WSTRB[0] = \<const1> ;
  assign S_AXI_BRESP[1] = \<const0> ;
  assign S_AXI_BRESP[0] = \<const0> ;
  assign S_AXI_RDATA[31] = \<const0> ;
  assign S_AXI_RDATA[30] = \<const0> ;
  assign S_AXI_RDATA[29] = \<const0> ;
  assign S_AXI_RDATA[28] = \<const0> ;
  assign S_AXI_RDATA[27] = \<const0> ;
  assign S_AXI_RDATA[26] = \<const0> ;
  assign S_AXI_RDATA[25] = \<const0> ;
  assign S_AXI_RDATA[24] = \<const0> ;
  assign S_AXI_RDATA[23] = \<const0> ;
  assign S_AXI_RDATA[22] = \<const0> ;
  assign S_AXI_RDATA[21] = \<const0> ;
  assign S_AXI_RDATA[20] = \<const0> ;
  assign S_AXI_RDATA[19] = \<const0> ;
  assign S_AXI_RDATA[18] = \<const0> ;
  assign S_AXI_RDATA[17] = \<const0> ;
  assign S_AXI_RDATA[16] = \<const0> ;
  assign S_AXI_RDATA[15] = \<const0> ;
  assign S_AXI_RDATA[14] = \<const0> ;
  assign S_AXI_RDATA[13] = \<const0> ;
  assign S_AXI_RDATA[12] = \<const0> ;
  assign S_AXI_RDATA[11] = \<const0> ;
  assign S_AXI_RDATA[10] = \<const0> ;
  assign S_AXI_RDATA[9] = \<const0> ;
  assign S_AXI_RDATA[8] = \<const0> ;
  assign S_AXI_RDATA[7] = \<const0> ;
  assign S_AXI_RDATA[6] = \<const0> ;
  assign S_AXI_RDATA[5] = \<const0> ;
  assign S_AXI_RDATA[4] = \<const0> ;
  assign S_AXI_RDATA[3] = \<const0> ;
  assign S_AXI_RDATA[2] = \<const0> ;
  assign S_AXI_RDATA[1] = \<const0> ;
  assign S_AXI_RDATA[0] = \<const0> ;
  assign S_AXI_RRESP[1] = \<const0> ;
  assign S_AXI_RRESP[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller inst
       (.M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_AWVALID(M_AXI_AWVALID),
        .M_AXI_BREADY(M_AXI_BREADY),
        .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_WREADY(M_AXI_WREADY),
        .M_AXI_WVALID(M_AXI_WVALID),
        .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_BVALID_reg(S_AXI_BVALID),
        .S_AXI_RREADY(S_AXI_RREADY),
        .S_AXI_RVALID(S_AXI_RVALID),
        .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_WVALID(S_AXI_WVALID),
        .aclk(aclk),
        .aresetn(aresetn));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller
   (M_AXI_WVALID,
    M_AXI_AWVALID,
    S_AXI_WREADY,
    S_AXI_AWREADY,
    S_AXI_ARREADY,
    S_AXI_RVALID,
    S_AXI_BVALID_reg,
    M_AXI_BREADY,
    M_AXI_WREADY,
    M_AXI_AWREADY,
    M_AXI_BVALID,
    aclk,
    S_AXI_WVALID,
    S_AXI_AWVALID,
    S_AXI_RREADY,
    S_AXI_ARVALID,
    S_AXI_BREADY,
    aresetn);
  output M_AXI_WVALID;
  output M_AXI_AWVALID;
  output S_AXI_WREADY;
  output S_AXI_AWREADY;
  output S_AXI_ARREADY;
  output S_AXI_RVALID;
  output S_AXI_BVALID_reg;
  output M_AXI_BREADY;
  input M_AXI_WREADY;
  input M_AXI_AWREADY;
  input M_AXI_BVALID;
  input aclk;
  input S_AXI_WVALID;
  input S_AXI_AWVALID;
  input S_AXI_RREADY;
  input S_AXI_ARVALID;
  input S_AXI_BREADY;
  input aresetn;

  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_BREADY;
  wire M_AXI_BVALID;
  wire M_AXI_WREADY;
  wire M_AXI_WVALID;
  wire S_AXI_ARREADY;
  wire S_AXI_ARVALID;
  wire S_AXI_AWREADY;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID_reg;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID;
  wire S_AXI_WREADY;
  wire S_AXI_WVALID;
  wire aclk;
  wire aresetn;
  wire slave1_n_1;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master master1
       (.M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_AWVALID(M_AXI_AWVALID),
        .M_AXI_BREADY(M_AXI_BREADY),
        .M_AXI_BREADY_reg_0(slave1_n_1),
        .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_WREADY(M_AXI_WREADY),
        .M_AXI_WVALID(M_AXI_WVALID),
        .aclk(aclk));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave slave1
       (.S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_BVALID_reg_0(S_AXI_BVALID_reg),
        .S_AXI_RREADY(S_AXI_RREADY),
        .S_AXI_RVALID(S_AXI_RVALID),
        .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_WVALID(S_AXI_WVALID),
        .aclk(aclk),
        .aresetn(aresetn),
        .aresetn_0(slave1_n_1));
endmodule
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

endmodule
`endif
