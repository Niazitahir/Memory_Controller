// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
// Date        : Tue Mar 10 22:45:20 2026
// Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_1_sim_netlist.v
// Design      : design_1_mem_controller_0_1
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* ST_AWBRAM = "2'b11" *) (* ST_AW_W = "2'b00" *) (* ST_B = "2'b01" *) 
(* ST_DONE = "2'b10" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master
   (bram_en,
    bram_addr,
    bram_din,
    bram_we,
    bram_dout,
    bram_ready,
    aclk,
    aresetn,
    M_AXI_AWADDR,
    M_AXI_AWVALID,
    M_AXI_AWREADY,
    M_AXI_WDATA,
    M_AXI_WSTRB,
    M_AXI_WVALID,
    M_AXI_WREADY,
    M_AXI_BRESP,
    M_AXI_BVALID,
    M_AXI_BREADY,
    value_out,
    init_write,
    addr_in,
    state_led);
  input bram_en;
  input [31:0]bram_addr;
  input [31:0]bram_din;
  input [3:0]bram_we;
  output [31:0]bram_dout;
  output bram_ready;
  input aclk;
  input aresetn;
  output [31:0]M_AXI_AWADDR;
  output M_AXI_AWVALID;
  input M_AXI_AWREADY;
  output [31:0]M_AXI_WDATA;
  output [3:0]M_AXI_WSTRB;
  output M_AXI_WVALID;
  input M_AXI_WREADY;
  (* mark_debug = "true" *) input [1:0]M_AXI_BRESP;
  input M_AXI_BVALID;
  output M_AXI_BREADY;
  input [31:0]value_out;
  input init_write;
  input [31:0]addr_in;
  output [1:0]state_led;

  wire \<const0> ;
  wire \<const1> ;
  wire \FSM_sequential_state[0]_i_1_n_0 ;
  wire \FSM_sequential_state[1]_i_1_n_0 ;
  wire [31:0]M_AXI_AWADDR;
  wire \M_AXI_AWADDR[31]_i_1_n_0 ;
  wire \M_AXI_AWADDR[31]_i_2_n_0 ;
  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_AWVALID_i_1_n_0;
  wire M_AXI_BREADY0;
  (* MARK_DEBUG *) wire [1:0]M_AXI_BRESP;
  wire M_AXI_BVALID;
  wire [31:0]M_AXI_WDATA;
  wire \M_AXI_WDATA[0]_i_1_n_0 ;
  wire \M_AXI_WDATA[10]_i_1_n_0 ;
  wire \M_AXI_WDATA[11]_i_1_n_0 ;
  wire \M_AXI_WDATA[12]_i_1_n_0 ;
  wire \M_AXI_WDATA[13]_i_1_n_0 ;
  wire \M_AXI_WDATA[14]_i_1_n_0 ;
  wire \M_AXI_WDATA[15]_i_1_n_0 ;
  wire \M_AXI_WDATA[16]_i_1_n_0 ;
  wire \M_AXI_WDATA[17]_i_1_n_0 ;
  wire \M_AXI_WDATA[18]_i_1_n_0 ;
  wire \M_AXI_WDATA[19]_i_1_n_0 ;
  wire \M_AXI_WDATA[1]_i_1_n_0 ;
  wire \M_AXI_WDATA[20]_i_1_n_0 ;
  wire \M_AXI_WDATA[21]_i_1_n_0 ;
  wire \M_AXI_WDATA[22]_i_1_n_0 ;
  wire \M_AXI_WDATA[23]_i_1_n_0 ;
  wire \M_AXI_WDATA[24]_i_1_n_0 ;
  wire \M_AXI_WDATA[25]_i_1_n_0 ;
  wire \M_AXI_WDATA[26]_i_1_n_0 ;
  wire \M_AXI_WDATA[27]_i_1_n_0 ;
  wire \M_AXI_WDATA[28]_i_1_n_0 ;
  wire \M_AXI_WDATA[29]_i_1_n_0 ;
  wire \M_AXI_WDATA[2]_i_1_n_0 ;
  wire \M_AXI_WDATA[30]_i_1_n_0 ;
  wire \M_AXI_WDATA[31]_i_1_n_0 ;
  wire \M_AXI_WDATA[3]_i_1_n_0 ;
  wire \M_AXI_WDATA[4]_i_1_n_0 ;
  wire \M_AXI_WDATA[5]_i_1_n_0 ;
  wire \M_AXI_WDATA[6]_i_1_n_0 ;
  wire \M_AXI_WDATA[7]_i_1_n_0 ;
  wire \M_AXI_WDATA[8]_i_1_n_0 ;
  wire \M_AXI_WDATA[9]_i_1_n_0 ;
  wire M_AXI_WREADY;
  wire [3:0]M_AXI_WSTRB;
  wire \M_AXI_WSTRB[0]_i_1_n_0 ;
  wire \M_AXI_WSTRB[1]_i_1_n_0 ;
  wire \M_AXI_WSTRB[2]_i_1_n_0 ;
  wire \M_AXI_WSTRB[3]_i_1_n_0 ;
  wire M_AXI_WVALID;
  wire M_AXI_WVALID_i_1_n_0;
  wire aclk;
  wire [31:0]addr_in;
  wire aresetn;
  wire [31:0]bram_addr;
  wire [31:0]bram_din;
  wire bram_en;
  wire bram_ready;
  wire bram_ready_i_1_n_0;
  wire [3:0]bram_we;
  wire \bram_we_latched[3]_i_1_n_0 ;
  wire \bram_we_latched_reg_n_0_[0] ;
  wire \bram_we_latched_reg_n_0_[1] ;
  wire \bram_we_latched_reg_n_0_[2] ;
  wire \bram_we_latched_reg_n_0_[3] ;
  wire init_write;
  wire [31:0]latch_addr;
  wire \latch_addr_reg_n_0_[0] ;
  wire \latch_addr_reg_n_0_[10] ;
  wire \latch_addr_reg_n_0_[11] ;
  wire \latch_addr_reg_n_0_[12] ;
  wire \latch_addr_reg_n_0_[13] ;
  wire \latch_addr_reg_n_0_[14] ;
  wire \latch_addr_reg_n_0_[15] ;
  wire \latch_addr_reg_n_0_[16] ;
  wire \latch_addr_reg_n_0_[17] ;
  wire \latch_addr_reg_n_0_[18] ;
  wire \latch_addr_reg_n_0_[19] ;
  wire \latch_addr_reg_n_0_[1] ;
  wire \latch_addr_reg_n_0_[20] ;
  wire \latch_addr_reg_n_0_[21] ;
  wire \latch_addr_reg_n_0_[22] ;
  wire \latch_addr_reg_n_0_[23] ;
  wire \latch_addr_reg_n_0_[24] ;
  wire \latch_addr_reg_n_0_[25] ;
  wire \latch_addr_reg_n_0_[26] ;
  wire \latch_addr_reg_n_0_[27] ;
  wire \latch_addr_reg_n_0_[28] ;
  wire \latch_addr_reg_n_0_[29] ;
  wire \latch_addr_reg_n_0_[2] ;
  wire \latch_addr_reg_n_0_[30] ;
  wire \latch_addr_reg_n_0_[31] ;
  wire \latch_addr_reg_n_0_[3] ;
  wire \latch_addr_reg_n_0_[4] ;
  wire \latch_addr_reg_n_0_[5] ;
  wire \latch_addr_reg_n_0_[6] ;
  wire \latch_addr_reg_n_0_[7] ;
  wire \latch_addr_reg_n_0_[8] ;
  wire \latch_addr_reg_n_0_[9] ;
  wire [1:0]state;
  wire [1:0]state_led;
  wire \state_led[1]_i_1_n_0 ;
  wire [31:0]value_out;

  assign M_AXI_BREADY = \<const1> ;
  assign bram_dout[31] = \<const0> ;
  assign bram_dout[30] = \<const0> ;
  assign bram_dout[29] = \<const0> ;
  assign bram_dout[28] = \<const0> ;
  assign bram_dout[27] = \<const0> ;
  assign bram_dout[26] = \<const0> ;
  assign bram_dout[25] = \<const0> ;
  assign bram_dout[24] = \<const0> ;
  assign bram_dout[23] = \<const0> ;
  assign bram_dout[22] = \<const0> ;
  assign bram_dout[21] = \<const0> ;
  assign bram_dout[20] = \<const0> ;
  assign bram_dout[19] = \<const0> ;
  assign bram_dout[18] = \<const0> ;
  assign bram_dout[17] = \<const0> ;
  assign bram_dout[16] = \<const0> ;
  assign bram_dout[15] = \<const0> ;
  assign bram_dout[14] = \<const0> ;
  assign bram_dout[13] = \<const0> ;
  assign bram_dout[12] = \<const0> ;
  assign bram_dout[11] = \<const0> ;
  assign bram_dout[10] = \<const0> ;
  assign bram_dout[9] = \<const0> ;
  assign bram_dout[8] = \<const0> ;
  assign bram_dout[7] = \<const0> ;
  assign bram_dout[6] = \<const0> ;
  assign bram_dout[5] = \<const0> ;
  assign bram_dout[4] = \<const0> ;
  assign bram_dout[3] = \<const0> ;
  assign bram_dout[2] = \<const0> ;
  assign bram_dout[1] = \<const0> ;
  assign bram_dout[0] = \<const0> ;
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h5FFC5F0C)) 
    \FSM_sequential_state[0]_i_1 
       (.I0(M_AXI_BVALID),
        .I1(bram_en),
        .I2(state[1]),
        .I3(state[0]),
        .I4(M_AXI_BREADY0),
        .O(\FSM_sequential_state[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h55FFFF0C5500FF0C)) 
    \FSM_sequential_state[1]_i_1 
       (.I0(M_AXI_BVALID),
        .I1(init_write),
        .I2(bram_en),
        .I3(state[1]),
        .I4(state[0]),
        .I5(M_AXI_BREADY0),
        .O(\FSM_sequential_state[1]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h8000)) 
    \FSM_sequential_state[1]_i_2 
       (.I0(M_AXI_AWREADY),
        .I1(M_AXI_AWVALID),
        .I2(M_AXI_WREADY),
        .I3(M_AXI_WVALID),
        .O(M_AXI_BREADY0));
  (* FSM_ENCODED_STATES = "ST_B:11,ST_DONE:00,ST_AW_W:10,ST_AWBRAM:01" *) 
  FDCE \FSM_sequential_state_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\FSM_sequential_state[0]_i_1_n_0 ),
        .Q(state[0]));
  (* FSM_ENCODED_STATES = "ST_B:11,ST_DONE:00,ST_AW_W:10,ST_AWBRAM:01" *) 
  FDCE \FSM_sequential_state_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\FSM_sequential_state[1]_i_1_n_0 ),
        .Q(state[1]));
  GND GND
       (.G(\<const0> ));
  LUT2 #(
    .INIT(4'h6)) 
    \M_AXI_AWADDR[31]_i_1 
       (.I0(state[0]),
        .I1(state[1]),
        .O(\M_AXI_AWADDR[31]_i_1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \M_AXI_AWADDR[31]_i_2 
       (.I0(aresetn),
        .O(\M_AXI_AWADDR[31]_i_2_n_0 ));
  FDCE \M_AXI_AWADDR_reg[0] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[0] ),
        .Q(M_AXI_AWADDR[0]));
  FDCE \M_AXI_AWADDR_reg[10] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[10] ),
        .Q(M_AXI_AWADDR[10]));
  FDCE \M_AXI_AWADDR_reg[11] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[11] ),
        .Q(M_AXI_AWADDR[11]));
  FDCE \M_AXI_AWADDR_reg[12] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[12] ),
        .Q(M_AXI_AWADDR[12]));
  FDCE \M_AXI_AWADDR_reg[13] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[13] ),
        .Q(M_AXI_AWADDR[13]));
  FDCE \M_AXI_AWADDR_reg[14] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[14] ),
        .Q(M_AXI_AWADDR[14]));
  FDCE \M_AXI_AWADDR_reg[15] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[15] ),
        .Q(M_AXI_AWADDR[15]));
  FDCE \M_AXI_AWADDR_reg[16] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[16] ),
        .Q(M_AXI_AWADDR[16]));
  FDCE \M_AXI_AWADDR_reg[17] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[17] ),
        .Q(M_AXI_AWADDR[17]));
  FDCE \M_AXI_AWADDR_reg[18] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[18] ),
        .Q(M_AXI_AWADDR[18]));
  FDCE \M_AXI_AWADDR_reg[19] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[19] ),
        .Q(M_AXI_AWADDR[19]));
  FDCE \M_AXI_AWADDR_reg[1] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[1] ),
        .Q(M_AXI_AWADDR[1]));
  FDCE \M_AXI_AWADDR_reg[20] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[20] ),
        .Q(M_AXI_AWADDR[20]));
  FDCE \M_AXI_AWADDR_reg[21] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[21] ),
        .Q(M_AXI_AWADDR[21]));
  FDCE \M_AXI_AWADDR_reg[22] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[22] ),
        .Q(M_AXI_AWADDR[22]));
  FDCE \M_AXI_AWADDR_reg[23] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[23] ),
        .Q(M_AXI_AWADDR[23]));
  FDCE \M_AXI_AWADDR_reg[24] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[24] ),
        .Q(M_AXI_AWADDR[24]));
  FDCE \M_AXI_AWADDR_reg[25] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[25] ),
        .Q(M_AXI_AWADDR[25]));
  FDCE \M_AXI_AWADDR_reg[26] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[26] ),
        .Q(M_AXI_AWADDR[26]));
  FDCE \M_AXI_AWADDR_reg[27] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[27] ),
        .Q(M_AXI_AWADDR[27]));
  FDCE \M_AXI_AWADDR_reg[28] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[28] ),
        .Q(M_AXI_AWADDR[28]));
  FDCE \M_AXI_AWADDR_reg[29] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[29] ),
        .Q(M_AXI_AWADDR[29]));
  FDCE \M_AXI_AWADDR_reg[2] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[2] ),
        .Q(M_AXI_AWADDR[2]));
  FDCE \M_AXI_AWADDR_reg[30] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[30] ),
        .Q(M_AXI_AWADDR[30]));
  FDCE \M_AXI_AWADDR_reg[31] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[31] ),
        .Q(M_AXI_AWADDR[31]));
  FDCE \M_AXI_AWADDR_reg[3] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[3] ),
        .Q(M_AXI_AWADDR[3]));
  FDCE \M_AXI_AWADDR_reg[4] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[4] ),
        .Q(M_AXI_AWADDR[4]));
  FDCE \M_AXI_AWADDR_reg[5] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[5] ),
        .Q(M_AXI_AWADDR[5]));
  FDCE \M_AXI_AWADDR_reg[6] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[6] ),
        .Q(M_AXI_AWADDR[6]));
  FDCE \M_AXI_AWADDR_reg[7] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[7] ),
        .Q(M_AXI_AWADDR[7]));
  FDCE \M_AXI_AWADDR_reg[8] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[8] ),
        .Q(M_AXI_AWADDR[8]));
  FDCE \M_AXI_AWADDR_reg[9] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\latch_addr_reg_n_0_[9] ),
        .Q(M_AXI_AWADDR[9]));
  LUT4 #(
    .INIT(16'hA77A)) 
    M_AXI_AWVALID_i_1
       (.I0(M_AXI_AWVALID),
        .I1(M_AXI_AWREADY),
        .I2(state[1]),
        .I3(state[0]),
        .O(M_AXI_AWVALID_i_1_n_0));
  FDCE M_AXI_AWVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(M_AXI_AWVALID_i_1_n_0),
        .Q(M_AXI_AWVALID));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[0]_i_1 
       (.I0(value_out[0]),
        .I1(bram_din[0]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[10]_i_1 
       (.I0(value_out[10]),
        .I1(bram_din[10]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[10]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[11]_i_1 
       (.I0(value_out[11]),
        .I1(bram_din[11]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[11]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[12]_i_1 
       (.I0(value_out[12]),
        .I1(bram_din[12]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[12]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[13]_i_1 
       (.I0(value_out[13]),
        .I1(bram_din[13]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[13]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[14]_i_1 
       (.I0(value_out[14]),
        .I1(bram_din[14]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[14]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[15]_i_1 
       (.I0(value_out[15]),
        .I1(bram_din[15]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[15]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[16]_i_1 
       (.I0(value_out[16]),
        .I1(bram_din[16]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[16]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[17]_i_1 
       (.I0(value_out[17]),
        .I1(bram_din[17]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[17]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[18]_i_1 
       (.I0(value_out[18]),
        .I1(bram_din[18]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[18]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[19]_i_1 
       (.I0(value_out[19]),
        .I1(bram_din[19]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[19]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[1]_i_1 
       (.I0(value_out[1]),
        .I1(bram_din[1]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[20]_i_1 
       (.I0(value_out[20]),
        .I1(bram_din[20]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[20]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[21]_i_1 
       (.I0(value_out[21]),
        .I1(bram_din[21]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[21]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[22]_i_1 
       (.I0(value_out[22]),
        .I1(bram_din[22]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[22]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[23]_i_1 
       (.I0(value_out[23]),
        .I1(bram_din[23]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[23]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[24]_i_1 
       (.I0(value_out[24]),
        .I1(bram_din[24]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[24]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[25]_i_1 
       (.I0(value_out[25]),
        .I1(bram_din[25]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[25]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[26]_i_1 
       (.I0(value_out[26]),
        .I1(bram_din[26]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[26]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[27]_i_1 
       (.I0(value_out[27]),
        .I1(bram_din[27]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[27]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[28]_i_1 
       (.I0(value_out[28]),
        .I1(bram_din[28]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[28]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[29]_i_1 
       (.I0(value_out[29]),
        .I1(bram_din[29]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[29]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[2]_i_1 
       (.I0(value_out[2]),
        .I1(bram_din[2]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[30]_i_1 
       (.I0(value_out[30]),
        .I1(bram_din[30]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[30]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[31]_i_1 
       (.I0(value_out[31]),
        .I1(bram_din[31]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[31]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[3]_i_1 
       (.I0(value_out[3]),
        .I1(bram_din[3]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[4]_i_1 
       (.I0(value_out[4]),
        .I1(bram_din[4]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[5]_i_1 
       (.I0(value_out[5]),
        .I1(bram_din[5]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[6]_i_1 
       (.I0(value_out[6]),
        .I1(bram_din[6]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[7]_i_1 
       (.I0(value_out[7]),
        .I1(bram_din[7]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[8]_i_1 
       (.I0(value_out[8]),
        .I1(bram_din[8]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[8]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \M_AXI_WDATA[9]_i_1 
       (.I0(value_out[9]),
        .I1(bram_din[9]),
        .I2(state[1]),
        .O(\M_AXI_WDATA[9]_i_1_n_0 ));
  FDCE \M_AXI_WDATA_reg[0] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[0]_i_1_n_0 ),
        .Q(M_AXI_WDATA[0]));
  FDCE \M_AXI_WDATA_reg[10] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[10]_i_1_n_0 ),
        .Q(M_AXI_WDATA[10]));
  FDCE \M_AXI_WDATA_reg[11] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[11]_i_1_n_0 ),
        .Q(M_AXI_WDATA[11]));
  FDCE \M_AXI_WDATA_reg[12] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[12]_i_1_n_0 ),
        .Q(M_AXI_WDATA[12]));
  FDCE \M_AXI_WDATA_reg[13] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[13]_i_1_n_0 ),
        .Q(M_AXI_WDATA[13]));
  FDCE \M_AXI_WDATA_reg[14] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[14]_i_1_n_0 ),
        .Q(M_AXI_WDATA[14]));
  FDCE \M_AXI_WDATA_reg[15] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[15]_i_1_n_0 ),
        .Q(M_AXI_WDATA[15]));
  FDCE \M_AXI_WDATA_reg[16] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[16]_i_1_n_0 ),
        .Q(M_AXI_WDATA[16]));
  FDCE \M_AXI_WDATA_reg[17] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[17]_i_1_n_0 ),
        .Q(M_AXI_WDATA[17]));
  FDCE \M_AXI_WDATA_reg[18] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[18]_i_1_n_0 ),
        .Q(M_AXI_WDATA[18]));
  FDCE \M_AXI_WDATA_reg[19] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[19]_i_1_n_0 ),
        .Q(M_AXI_WDATA[19]));
  FDCE \M_AXI_WDATA_reg[1] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[1]_i_1_n_0 ),
        .Q(M_AXI_WDATA[1]));
  FDCE \M_AXI_WDATA_reg[20] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[20]_i_1_n_0 ),
        .Q(M_AXI_WDATA[20]));
  FDCE \M_AXI_WDATA_reg[21] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[21]_i_1_n_0 ),
        .Q(M_AXI_WDATA[21]));
  FDCE \M_AXI_WDATA_reg[22] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[22]_i_1_n_0 ),
        .Q(M_AXI_WDATA[22]));
  FDCE \M_AXI_WDATA_reg[23] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[23]_i_1_n_0 ),
        .Q(M_AXI_WDATA[23]));
  FDCE \M_AXI_WDATA_reg[24] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[24]_i_1_n_0 ),
        .Q(M_AXI_WDATA[24]));
  FDCE \M_AXI_WDATA_reg[25] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[25]_i_1_n_0 ),
        .Q(M_AXI_WDATA[25]));
  FDCE \M_AXI_WDATA_reg[26] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[26]_i_1_n_0 ),
        .Q(M_AXI_WDATA[26]));
  FDCE \M_AXI_WDATA_reg[27] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[27]_i_1_n_0 ),
        .Q(M_AXI_WDATA[27]));
  FDCE \M_AXI_WDATA_reg[28] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[28]_i_1_n_0 ),
        .Q(M_AXI_WDATA[28]));
  FDCE \M_AXI_WDATA_reg[29] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[29]_i_1_n_0 ),
        .Q(M_AXI_WDATA[29]));
  FDCE \M_AXI_WDATA_reg[2] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[2]_i_1_n_0 ),
        .Q(M_AXI_WDATA[2]));
  FDCE \M_AXI_WDATA_reg[30] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[30]_i_1_n_0 ),
        .Q(M_AXI_WDATA[30]));
  FDCE \M_AXI_WDATA_reg[31] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[31]_i_1_n_0 ),
        .Q(M_AXI_WDATA[31]));
  FDCE \M_AXI_WDATA_reg[3] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[3]_i_1_n_0 ),
        .Q(M_AXI_WDATA[3]));
  FDCE \M_AXI_WDATA_reg[4] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[4]_i_1_n_0 ),
        .Q(M_AXI_WDATA[4]));
  FDCE \M_AXI_WDATA_reg[5] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[5]_i_1_n_0 ),
        .Q(M_AXI_WDATA[5]));
  FDCE \M_AXI_WDATA_reg[6] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[6]_i_1_n_0 ),
        .Q(M_AXI_WDATA[6]));
  FDCE \M_AXI_WDATA_reg[7] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[7]_i_1_n_0 ),
        .Q(M_AXI_WDATA[7]));
  FDCE \M_AXI_WDATA_reg[8] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[8]_i_1_n_0 ),
        .Q(M_AXI_WDATA[8]));
  FDCE \M_AXI_WDATA_reg[9] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\M_AXI_WDATA[9]_i_1_n_0 ),
        .Q(M_AXI_WDATA[9]));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \M_AXI_WSTRB[0]_i_1 
       (.I0(state[1]),
        .I1(\bram_we_latched_reg_n_0_[0] ),
        .O(\M_AXI_WSTRB[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \M_AXI_WSTRB[1]_i_1 
       (.I0(state[1]),
        .I1(\bram_we_latched_reg_n_0_[1] ),
        .O(\M_AXI_WSTRB[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \M_AXI_WSTRB[2]_i_1 
       (.I0(state[1]),
        .I1(\bram_we_latched_reg_n_0_[2] ),
        .O(\M_AXI_WSTRB[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \M_AXI_WSTRB[3]_i_1 
       (.I0(state[1]),
        .I1(\bram_we_latched_reg_n_0_[3] ),
        .O(\M_AXI_WSTRB[3]_i_1_n_0 ));
  FDPE \M_AXI_WSTRB_reg[0] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .D(\M_AXI_WSTRB[0]_i_1_n_0 ),
        .PRE(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .Q(M_AXI_WSTRB[0]));
  FDPE \M_AXI_WSTRB_reg[1] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .D(\M_AXI_WSTRB[1]_i_1_n_0 ),
        .PRE(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .Q(M_AXI_WSTRB[1]));
  FDPE \M_AXI_WSTRB_reg[2] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .D(\M_AXI_WSTRB[2]_i_1_n_0 ),
        .PRE(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .Q(M_AXI_WSTRB[2]));
  FDPE \M_AXI_WSTRB_reg[3] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .D(\M_AXI_WSTRB[3]_i_1_n_0 ),
        .PRE(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .Q(M_AXI_WSTRB[3]));
  LUT4 #(
    .INIT(16'hA77A)) 
    M_AXI_WVALID_i_1
       (.I0(M_AXI_WVALID),
        .I1(M_AXI_WREADY),
        .I2(state[1]),
        .I3(state[0]),
        .O(M_AXI_WVALID_i_1_n_0));
  FDCE M_AXI_WVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(M_AXI_WVALID_i_1_n_0),
        .Q(M_AXI_WVALID));
  VCC VCC
       (.P(\<const1> ));
  LUT4 #(
    .INIT(16'hF704)) 
    bram_ready_i_1
       (.I0(state[0]),
        .I1(aresetn),
        .I2(state[1]),
        .I3(bram_ready),
        .O(bram_ready_i_1_n_0));
  FDRE bram_ready_reg
       (.C(aclk),
        .CE(1'b1),
        .D(bram_ready_i_1_n_0),
        .Q(bram_ready),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h20)) 
    \bram_we_latched[3]_i_1 
       (.I0(aresetn),
        .I1(state[1]),
        .I2(state[0]),
        .O(\bram_we_latched[3]_i_1_n_0 ));
  FDRE \bram_we_latched_reg[0] 
       (.C(aclk),
        .CE(\bram_we_latched[3]_i_1_n_0 ),
        .D(bram_we[0]),
        .Q(\bram_we_latched_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \bram_we_latched_reg[1] 
       (.C(aclk),
        .CE(\bram_we_latched[3]_i_1_n_0 ),
        .D(bram_we[1]),
        .Q(\bram_we_latched_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \bram_we_latched_reg[2] 
       (.C(aclk),
        .CE(\bram_we_latched[3]_i_1_n_0 ),
        .D(bram_we[2]),
        .Q(\bram_we_latched_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \bram_we_latched_reg[3] 
       (.C(aclk),
        .CE(\bram_we_latched[3]_i_1_n_0 ),
        .D(bram_we[3]),
        .Q(\bram_we_latched_reg_n_0_[3] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[0]_i_1 
       (.I0(addr_in[0]),
        .I1(bram_addr[0]),
        .I2(state[1]),
        .O(latch_addr[0]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[10]_i_1 
       (.I0(addr_in[10]),
        .I1(bram_addr[10]),
        .I2(state[1]),
        .O(latch_addr[10]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[11]_i_1 
       (.I0(addr_in[11]),
        .I1(bram_addr[11]),
        .I2(state[1]),
        .O(latch_addr[11]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[12]_i_1 
       (.I0(addr_in[12]),
        .I1(bram_addr[12]),
        .I2(state[1]),
        .O(latch_addr[12]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[13]_i_1 
       (.I0(addr_in[13]),
        .I1(bram_addr[13]),
        .I2(state[1]),
        .O(latch_addr[13]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[14]_i_1 
       (.I0(addr_in[14]),
        .I1(bram_addr[14]),
        .I2(state[1]),
        .O(latch_addr[14]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[15]_i_1 
       (.I0(addr_in[15]),
        .I1(bram_addr[15]),
        .I2(state[1]),
        .O(latch_addr[15]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[16]_i_1 
       (.I0(addr_in[16]),
        .I1(bram_addr[16]),
        .I2(state[1]),
        .O(latch_addr[16]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[17]_i_1 
       (.I0(addr_in[17]),
        .I1(bram_addr[17]),
        .I2(state[1]),
        .O(latch_addr[17]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[18]_i_1 
       (.I0(addr_in[18]),
        .I1(bram_addr[18]),
        .I2(state[1]),
        .O(latch_addr[18]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[19]_i_1 
       (.I0(addr_in[19]),
        .I1(bram_addr[19]),
        .I2(state[1]),
        .O(latch_addr[19]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[1]_i_1 
       (.I0(addr_in[1]),
        .I1(bram_addr[1]),
        .I2(state[1]),
        .O(latch_addr[1]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[20]_i_1 
       (.I0(addr_in[20]),
        .I1(bram_addr[20]),
        .I2(state[1]),
        .O(latch_addr[20]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[21]_i_1 
       (.I0(addr_in[21]),
        .I1(bram_addr[21]),
        .I2(state[1]),
        .O(latch_addr[21]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[22]_i_1 
       (.I0(addr_in[22]),
        .I1(bram_addr[22]),
        .I2(state[1]),
        .O(latch_addr[22]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[23]_i_1 
       (.I0(addr_in[23]),
        .I1(bram_addr[23]),
        .I2(state[1]),
        .O(latch_addr[23]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[24]_i_1 
       (.I0(addr_in[24]),
        .I1(bram_addr[24]),
        .I2(state[1]),
        .O(latch_addr[24]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[25]_i_1 
       (.I0(addr_in[25]),
        .I1(bram_addr[25]),
        .I2(state[1]),
        .O(latch_addr[25]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[26]_i_1 
       (.I0(addr_in[26]),
        .I1(bram_addr[26]),
        .I2(state[1]),
        .O(latch_addr[26]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[27]_i_1 
       (.I0(addr_in[27]),
        .I1(bram_addr[27]),
        .I2(state[1]),
        .O(latch_addr[27]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[28]_i_1 
       (.I0(addr_in[28]),
        .I1(bram_addr[28]),
        .I2(state[1]),
        .O(latch_addr[28]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[29]_i_1 
       (.I0(addr_in[29]),
        .I1(bram_addr[29]),
        .I2(state[1]),
        .O(latch_addr[29]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[2]_i_1 
       (.I0(addr_in[2]),
        .I1(bram_addr[2]),
        .I2(state[1]),
        .O(latch_addr[2]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[30]_i_1 
       (.I0(addr_in[30]),
        .I1(bram_addr[30]),
        .I2(state[1]),
        .O(latch_addr[30]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[31]_i_1 
       (.I0(addr_in[31]),
        .I1(bram_addr[31]),
        .I2(state[1]),
        .O(latch_addr[31]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[3]_i_1 
       (.I0(addr_in[3]),
        .I1(bram_addr[3]),
        .I2(state[1]),
        .O(latch_addr[3]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[4]_i_1 
       (.I0(addr_in[4]),
        .I1(bram_addr[4]),
        .I2(state[1]),
        .O(latch_addr[4]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[5]_i_1 
       (.I0(addr_in[5]),
        .I1(bram_addr[5]),
        .I2(state[1]),
        .O(latch_addr[5]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[6]_i_1 
       (.I0(addr_in[6]),
        .I1(bram_addr[6]),
        .I2(state[1]),
        .O(latch_addr[6]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[7]_i_1 
       (.I0(addr_in[7]),
        .I1(bram_addr[7]),
        .I2(state[1]),
        .O(latch_addr[7]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[8]_i_1 
       (.I0(addr_in[8]),
        .I1(bram_addr[8]),
        .I2(state[1]),
        .O(latch_addr[8]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \latch_addr[9]_i_1 
       (.I0(addr_in[9]),
        .I1(bram_addr[9]),
        .I2(state[1]),
        .O(latch_addr[9]));
  FDCE \latch_addr_reg[0] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[0]),
        .Q(\latch_addr_reg_n_0_[0] ));
  FDCE \latch_addr_reg[10] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[10]),
        .Q(\latch_addr_reg_n_0_[10] ));
  FDCE \latch_addr_reg[11] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[11]),
        .Q(\latch_addr_reg_n_0_[11] ));
  FDCE \latch_addr_reg[12] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[12]),
        .Q(\latch_addr_reg_n_0_[12] ));
  FDCE \latch_addr_reg[13] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[13]),
        .Q(\latch_addr_reg_n_0_[13] ));
  FDCE \latch_addr_reg[14] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[14]),
        .Q(\latch_addr_reg_n_0_[14] ));
  FDCE \latch_addr_reg[15] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[15]),
        .Q(\latch_addr_reg_n_0_[15] ));
  FDCE \latch_addr_reg[16] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[16]),
        .Q(\latch_addr_reg_n_0_[16] ));
  FDCE \latch_addr_reg[17] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[17]),
        .Q(\latch_addr_reg_n_0_[17] ));
  FDCE \latch_addr_reg[18] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[18]),
        .Q(\latch_addr_reg_n_0_[18] ));
  FDCE \latch_addr_reg[19] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[19]),
        .Q(\latch_addr_reg_n_0_[19] ));
  FDCE \latch_addr_reg[1] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[1]),
        .Q(\latch_addr_reg_n_0_[1] ));
  FDCE \latch_addr_reg[20] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[20]),
        .Q(\latch_addr_reg_n_0_[20] ));
  FDCE \latch_addr_reg[21] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[21]),
        .Q(\latch_addr_reg_n_0_[21] ));
  FDCE \latch_addr_reg[22] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[22]),
        .Q(\latch_addr_reg_n_0_[22] ));
  FDCE \latch_addr_reg[23] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[23]),
        .Q(\latch_addr_reg_n_0_[23] ));
  FDCE \latch_addr_reg[24] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[24]),
        .Q(\latch_addr_reg_n_0_[24] ));
  FDCE \latch_addr_reg[25] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[25]),
        .Q(\latch_addr_reg_n_0_[25] ));
  FDCE \latch_addr_reg[26] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[26]),
        .Q(\latch_addr_reg_n_0_[26] ));
  FDCE \latch_addr_reg[27] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[27]),
        .Q(\latch_addr_reg_n_0_[27] ));
  FDCE \latch_addr_reg[28] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[28]),
        .Q(\latch_addr_reg_n_0_[28] ));
  FDCE \latch_addr_reg[29] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[29]),
        .Q(\latch_addr_reg_n_0_[29] ));
  FDCE \latch_addr_reg[2] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[2]),
        .Q(\latch_addr_reg_n_0_[2] ));
  FDCE \latch_addr_reg[30] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[30]),
        .Q(\latch_addr_reg_n_0_[30] ));
  FDCE \latch_addr_reg[31] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[31]),
        .Q(\latch_addr_reg_n_0_[31] ));
  FDCE \latch_addr_reg[3] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[3]),
        .Q(\latch_addr_reg_n_0_[3] ));
  FDCE \latch_addr_reg[4] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[4]),
        .Q(\latch_addr_reg_n_0_[4] ));
  FDCE \latch_addr_reg[5] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[5]),
        .Q(\latch_addr_reg_n_0_[5] ));
  FDCE \latch_addr_reg[6] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[6]),
        .Q(\latch_addr_reg_n_0_[6] ));
  FDCE \latch_addr_reg[7] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[7]),
        .Q(\latch_addr_reg_n_0_[7] ));
  FDCE \latch_addr_reg[8] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[8]),
        .Q(\latch_addr_reg_n_0_[8] ));
  FDCE \latch_addr_reg[9] 
       (.C(aclk),
        .CE(\M_AXI_AWADDR[31]_i_1_n_0 ),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(latch_addr[9]),
        .Q(\latch_addr_reg_n_0_[9] ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \state_led[1]_i_1 
       (.I0(state[1]),
        .O(\state_led[1]_i_1_n_0 ));
  FDCE \state_led_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(state[0]),
        .Q(state_led[0]));
  FDCE \state_led_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(\M_AXI_AWADDR[31]_i_2_n_0 ),
        .D(\state_led[1]_i_1_n_0 ),
        .Q(state_led[1]));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave
   (init_write,
    aresetn_0,
    S_AXI_BVALID_reg_0,
    S_AXI_WREADY_reg_0,
    S_AXI_AWREADY_reg_0,
    S_AXI_RVALID_reg_0,
    S_AXI_ARREADY_reg_0,
    S_AXI_RDATA,
    aclk,
    S_AXI_ARVALID,
    S_AXI_BREADY,
    S_AXI_WVALID,
    S_AXI_AWVALID,
    S_AXI_RREADY,
    aresetn,
    S_AXI_AWADDR,
    S_AXI_WDATA,
    S_AXI_ARADDR);
  output init_write;
  output aresetn_0;
  output S_AXI_BVALID_reg_0;
  output S_AXI_WREADY_reg_0;
  output S_AXI_AWREADY_reg_0;
  output S_AXI_RVALID_reg_0;
  output S_AXI_ARREADY_reg_0;
  output [1:0]S_AXI_RDATA;
  input aclk;
  input S_AXI_ARVALID;
  input S_AXI_BREADY;
  input S_AXI_WVALID;
  input S_AXI_AWVALID;
  input S_AXI_RREADY;
  input aresetn;
  input [31:0]S_AXI_AWADDR;
  input [1:0]S_AXI_WDATA;
  input [31:0]S_AXI_ARADDR;

  wire [31:0]S_AXI_ARADDR;
  wire S_AXI_ARREADY_i_1_n_0;
  wire S_AXI_ARREADY_reg_0;
  wire S_AXI_ARVALID;
  wire [31:0]S_AXI_AWADDR;
  wire S_AXI_AWREADY_i_1_n_0;
  wire S_AXI_AWREADY_reg_0;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID_i_1_n_0;
  wire S_AXI_BVALID_reg_0;
  wire [1:0]S_AXI_RDATA;
  wire \S_AXI_RDATA[0]_i_1_n_0 ;
  wire \S_AXI_RDATA[1]_i_10_n_0 ;
  wire \S_AXI_RDATA[1]_i_11_n_0 ;
  wire \S_AXI_RDATA[1]_i_1_n_0 ;
  wire \S_AXI_RDATA[1]_i_2_n_0 ;
  wire \S_AXI_RDATA[1]_i_3_n_0 ;
  wire \S_AXI_RDATA[1]_i_4_n_0 ;
  wire \S_AXI_RDATA[1]_i_6_n_0 ;
  wire \S_AXI_RDATA[1]_i_7_n_0 ;
  wire \S_AXI_RDATA[1]_i_8_n_0 ;
  wire \S_AXI_RDATA[1]_i_9_n_0 ;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID0;
  wire S_AXI_RVALID_i_1_n_0;
  wire S_AXI_RVALID_reg_0;
  wire [1:0]S_AXI_WDATA;
  wire S_AXI_WREADY_i_1_n_0;
  wire S_AXI_WREADY_reg_0;
  wire S_AXI_WVALID;
  wire aclk;
  wire ar_latched_i_1_n_0;
  wire ar_latched_reg_n_0;
  wire [31:0]araddr_reg;
  wire \araddr_reg[31]_i_1_n_0 ;
  wire aresetn;
  wire aresetn_0;
  wire aw_latched_i_1_n_0;
  wire aw_latched_reg_n_0;
  wire [31:0]awaddr_reg;
  wire awaddr_reg_0;
  wire init_write;
  wire mem_fin_i_1_n_0;
  wire [1:0]mode;
  wire \mode[0]_i_1_n_0 ;
  wire \mode[1]_i_1_n_0 ;
  wire \mode[1]_i_2_n_0 ;
  wire \mode[1]_i_3_n_0 ;
  wire \mode[1]_i_4_n_0 ;
  wire \mode[1]_i_5_n_0 ;
  wire \mode[1]_i_6_n_0 ;
  wire \mode[1]_i_7_n_0 ;
  wire \mode[1]_i_8_n_0 ;
  wire w_latched_i_1_n_0;
  wire w_latched_reg_n_0;
  wire \wdata_reg[0]_i_1_n_0 ;
  wire \wdata_reg[1]_i_1_n_0 ;
  wire \wdata_reg_reg_n_0_[0] ;
  wire \wdata_reg_reg_n_0_[1] ;

  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT3 #(
    .INIT(8'h15)) 
    S_AXI_ARREADY_i_1
       (.I0(ar_latched_reg_n_0),
        .I1(S_AXI_ARREADY_reg_0),
        .I2(S_AXI_ARVALID),
        .O(S_AXI_ARREADY_i_1_n_0));
  FDCE S_AXI_ARREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_ARREADY_i_1_n_0),
        .Q(S_AXI_ARREADY_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT3 #(
    .INIT(8'h07)) 
    S_AXI_AWREADY_i_1
       (.I0(S_AXI_AWREADY_reg_0),
        .I1(S_AXI_AWVALID),
        .I2(aw_latched_reg_n_0),
        .O(S_AXI_AWREADY_i_1_n_0));
  FDCE S_AXI_AWREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_AWREADY_i_1_n_0),
        .Q(S_AXI_AWREADY_reg_0));
  LUT4 #(
    .INIT(16'h5C50)) 
    S_AXI_BVALID_i_1
       (.I0(S_AXI_BREADY),
        .I1(aw_latched_reg_n_0),
        .I2(S_AXI_BVALID_reg_0),
        .I3(w_latched_reg_n_0),
        .O(S_AXI_BVALID_i_1_n_0));
  FDCE S_AXI_BVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_BVALID_i_1_n_0),
        .Q(S_AXI_BVALID_reg_0));
  LUT6 #(
    .INIT(64'h0002FFFF00020000)) 
    \S_AXI_RDATA[0]_i_1 
       (.I0(mode[0]),
        .I1(\S_AXI_RDATA[1]_i_2_n_0 ),
        .I2(\S_AXI_RDATA[1]_i_3_n_0 ),
        .I3(\S_AXI_RDATA[1]_i_4_n_0 ),
        .I4(S_AXI_RVALID0),
        .I5(S_AXI_RDATA[0]),
        .O(\S_AXI_RDATA[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0002FFFF00020000)) 
    \S_AXI_RDATA[1]_i_1 
       (.I0(mode[1]),
        .I1(\S_AXI_RDATA[1]_i_2_n_0 ),
        .I2(\S_AXI_RDATA[1]_i_3_n_0 ),
        .I3(\S_AXI_RDATA[1]_i_4_n_0 ),
        .I4(S_AXI_RVALID0),
        .I5(S_AXI_RDATA[1]),
        .O(\S_AXI_RDATA[1]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_10 
       (.I0(araddr_reg[27]),
        .I1(araddr_reg[28]),
        .I2(araddr_reg[25]),
        .I3(araddr_reg[26]),
        .O(\S_AXI_RDATA[1]_i_10_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_11 
       (.I0(araddr_reg[19]),
        .I1(araddr_reg[20]),
        .I2(araddr_reg[17]),
        .I3(araddr_reg[18]),
        .O(\S_AXI_RDATA[1]_i_11_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_2 
       (.I0(\S_AXI_RDATA[1]_i_6_n_0 ),
        .I1(\S_AXI_RDATA[1]_i_7_n_0 ),
        .I2(\S_AXI_RDATA[1]_i_8_n_0 ),
        .I3(\S_AXI_RDATA[1]_i_9_n_0 ),
        .O(\S_AXI_RDATA[1]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'hFFFFFFFE)) 
    \S_AXI_RDATA[1]_i_3 
       (.I0(araddr_reg[30]),
        .I1(araddr_reg[29]),
        .I2(araddr_reg[1]),
        .I3(araddr_reg[0]),
        .I4(\S_AXI_RDATA[1]_i_10_n_0 ),
        .O(\S_AXI_RDATA[1]_i_3_n_0 ));
  LUT5 #(
    .INIT(32'hFFFFFFFE)) 
    \S_AXI_RDATA[1]_i_4 
       (.I0(araddr_reg[22]),
        .I1(araddr_reg[21]),
        .I2(araddr_reg[24]),
        .I3(araddr_reg[23]),
        .I4(\S_AXI_RDATA[1]_i_11_n_0 ),
        .O(\S_AXI_RDATA[1]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \S_AXI_RDATA[1]_i_5 
       (.I0(ar_latched_reg_n_0),
        .I1(S_AXI_RVALID_reg_0),
        .O(S_AXI_RVALID0));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_6 
       (.I0(araddr_reg[11]),
        .I1(araddr_reg[12]),
        .I2(araddr_reg[9]),
        .I3(araddr_reg[10]),
        .O(\S_AXI_RDATA[1]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_7 
       (.I0(araddr_reg[15]),
        .I1(araddr_reg[16]),
        .I2(araddr_reg[13]),
        .I3(araddr_reg[14]),
        .O(\S_AXI_RDATA[1]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'hFFEF)) 
    \S_AXI_RDATA[1]_i_8 
       (.I0(araddr_reg[3]),
        .I1(araddr_reg[4]),
        .I2(araddr_reg[31]),
        .I3(araddr_reg[2]),
        .O(\S_AXI_RDATA[1]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \S_AXI_RDATA[1]_i_9 
       (.I0(araddr_reg[7]),
        .I1(araddr_reg[8]),
        .I2(araddr_reg[5]),
        .I3(araddr_reg[6]),
        .O(\S_AXI_RDATA[1]_i_9_n_0 ));
  FDCE \S_AXI_RDATA_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\S_AXI_RDATA[0]_i_1_n_0 ),
        .Q(S_AXI_RDATA[0]));
  FDCE \S_AXI_RDATA_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\S_AXI_RDATA[1]_i_1_n_0 ),
        .Q(S_AXI_RDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT3 #(
    .INIT(8'h74)) 
    S_AXI_RVALID_i_1
       (.I0(S_AXI_RREADY),
        .I1(S_AXI_RVALID_reg_0),
        .I2(ar_latched_reg_n_0),
        .O(S_AXI_RVALID_i_1_n_0));
  FDCE S_AXI_RVALID_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_RVALID_i_1_n_0),
        .Q(S_AXI_RVALID_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT3 #(
    .INIT(8'h13)) 
    S_AXI_WREADY_i_1
       (.I0(S_AXI_WREADY_reg_0),
        .I1(w_latched_reg_n_0),
        .I2(S_AXI_WVALID),
        .O(S_AXI_WREADY_i_1_n_0));
  FDCE S_AXI_WREADY_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(S_AXI_WREADY_i_1_n_0),
        .Q(S_AXI_WREADY_reg_0));
  LUT1 #(
    .INIT(2'h1)) 
    a_en_i_2
       (.I0(aresetn),
        .O(aresetn_0));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT4 #(
    .INIT(16'hB888)) 
    ar_latched_i_1
       (.I0(S_AXI_RVALID_reg_0),
        .I1(ar_latched_reg_n_0),
        .I2(S_AXI_ARREADY_reg_0),
        .I3(S_AXI_ARVALID),
        .O(ar_latched_i_1_n_0));
  FDCE ar_latched_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(ar_latched_i_1_n_0),
        .Q(ar_latched_reg_n_0));
  LUT2 #(
    .INIT(4'h8)) 
    \araddr_reg[31]_i_1 
       (.I0(S_AXI_ARVALID),
        .I1(S_AXI_ARREADY_reg_0),
        .O(\araddr_reg[31]_i_1_n_0 ));
  FDCE \araddr_reg_reg[0] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[0]),
        .Q(araddr_reg[0]));
  FDCE \araddr_reg_reg[10] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[10]),
        .Q(araddr_reg[10]));
  FDCE \araddr_reg_reg[11] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[11]),
        .Q(araddr_reg[11]));
  FDCE \araddr_reg_reg[12] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[12]),
        .Q(araddr_reg[12]));
  FDCE \araddr_reg_reg[13] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[13]),
        .Q(araddr_reg[13]));
  FDCE \araddr_reg_reg[14] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[14]),
        .Q(araddr_reg[14]));
  FDCE \araddr_reg_reg[15] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[15]),
        .Q(araddr_reg[15]));
  FDCE \araddr_reg_reg[16] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[16]),
        .Q(araddr_reg[16]));
  FDCE \araddr_reg_reg[17] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[17]),
        .Q(araddr_reg[17]));
  FDCE \araddr_reg_reg[18] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[18]),
        .Q(araddr_reg[18]));
  FDCE \araddr_reg_reg[19] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[19]),
        .Q(araddr_reg[19]));
  FDCE \araddr_reg_reg[1] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[1]),
        .Q(araddr_reg[1]));
  FDCE \araddr_reg_reg[20] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[20]),
        .Q(araddr_reg[20]));
  FDCE \araddr_reg_reg[21] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[21]),
        .Q(araddr_reg[21]));
  FDCE \araddr_reg_reg[22] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[22]),
        .Q(araddr_reg[22]));
  FDCE \araddr_reg_reg[23] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[23]),
        .Q(araddr_reg[23]));
  FDCE \araddr_reg_reg[24] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[24]),
        .Q(araddr_reg[24]));
  FDCE \araddr_reg_reg[25] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[25]),
        .Q(araddr_reg[25]));
  FDCE \araddr_reg_reg[26] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[26]),
        .Q(araddr_reg[26]));
  FDCE \araddr_reg_reg[27] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[27]),
        .Q(araddr_reg[27]));
  FDCE \araddr_reg_reg[28] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[28]),
        .Q(araddr_reg[28]));
  FDCE \araddr_reg_reg[29] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[29]),
        .Q(araddr_reg[29]));
  FDCE \araddr_reg_reg[2] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[2]),
        .Q(araddr_reg[2]));
  FDCE \araddr_reg_reg[30] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[30]),
        .Q(araddr_reg[30]));
  FDCE \araddr_reg_reg[31] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[31]),
        .Q(araddr_reg[31]));
  FDCE \araddr_reg_reg[3] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[3]),
        .Q(araddr_reg[3]));
  FDCE \araddr_reg_reg[4] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[4]),
        .Q(araddr_reg[4]));
  FDCE \araddr_reg_reg[5] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[5]),
        .Q(araddr_reg[5]));
  FDCE \araddr_reg_reg[6] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[6]),
        .Q(araddr_reg[6]));
  FDCE \araddr_reg_reg[7] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[7]),
        .Q(araddr_reg[7]));
  FDCE \araddr_reg_reg[8] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[8]),
        .Q(araddr_reg[8]));
  FDCE \araddr_reg_reg[9] 
       (.C(aclk),
        .CE(\araddr_reg[31]_i_1_n_0 ),
        .CLR(aresetn_0),
        .D(S_AXI_ARADDR[9]),
        .Q(araddr_reg[9]));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT5 #(
    .INIT(32'hF808F8F8)) 
    aw_latched_i_1
       (.I0(S_AXI_AWVALID),
        .I1(S_AXI_AWREADY_reg_0),
        .I2(aw_latched_reg_n_0),
        .I3(S_AXI_BVALID_reg_0),
        .I4(w_latched_reg_n_0),
        .O(aw_latched_i_1_n_0));
  FDCE aw_latched_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(aw_latched_i_1_n_0),
        .Q(aw_latched_reg_n_0));
  LUT2 #(
    .INIT(4'h8)) 
    \awaddr_reg[31]_i_1 
       (.I0(S_AXI_AWREADY_reg_0),
        .I1(S_AXI_AWVALID),
        .O(awaddr_reg_0));
  FDCE \awaddr_reg_reg[0] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[0]),
        .Q(awaddr_reg[0]));
  FDCE \awaddr_reg_reg[10] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[10]),
        .Q(awaddr_reg[10]));
  FDCE \awaddr_reg_reg[11] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[11]),
        .Q(awaddr_reg[11]));
  FDCE \awaddr_reg_reg[12] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[12]),
        .Q(awaddr_reg[12]));
  FDCE \awaddr_reg_reg[13] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[13]),
        .Q(awaddr_reg[13]));
  FDCE \awaddr_reg_reg[14] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[14]),
        .Q(awaddr_reg[14]));
  FDCE \awaddr_reg_reg[15] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[15]),
        .Q(awaddr_reg[15]));
  FDCE \awaddr_reg_reg[16] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[16]),
        .Q(awaddr_reg[16]));
  FDCE \awaddr_reg_reg[17] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[17]),
        .Q(awaddr_reg[17]));
  FDCE \awaddr_reg_reg[18] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[18]),
        .Q(awaddr_reg[18]));
  FDCE \awaddr_reg_reg[19] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[19]),
        .Q(awaddr_reg[19]));
  FDCE \awaddr_reg_reg[1] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[1]),
        .Q(awaddr_reg[1]));
  FDCE \awaddr_reg_reg[20] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[20]),
        .Q(awaddr_reg[20]));
  FDCE \awaddr_reg_reg[21] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[21]),
        .Q(awaddr_reg[21]));
  FDCE \awaddr_reg_reg[22] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[22]),
        .Q(awaddr_reg[22]));
  FDCE \awaddr_reg_reg[23] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[23]),
        .Q(awaddr_reg[23]));
  FDCE \awaddr_reg_reg[24] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[24]),
        .Q(awaddr_reg[24]));
  FDCE \awaddr_reg_reg[25] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[25]),
        .Q(awaddr_reg[25]));
  FDCE \awaddr_reg_reg[26] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[26]),
        .Q(awaddr_reg[26]));
  FDCE \awaddr_reg_reg[27] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[27]),
        .Q(awaddr_reg[27]));
  FDCE \awaddr_reg_reg[28] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[28]),
        .Q(awaddr_reg[28]));
  FDCE \awaddr_reg_reg[29] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[29]),
        .Q(awaddr_reg[29]));
  FDCE \awaddr_reg_reg[2] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[2]),
        .Q(awaddr_reg[2]));
  FDCE \awaddr_reg_reg[30] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[30]),
        .Q(awaddr_reg[30]));
  FDCE \awaddr_reg_reg[31] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[31]),
        .Q(awaddr_reg[31]));
  FDCE \awaddr_reg_reg[3] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[3]),
        .Q(awaddr_reg[3]));
  FDCE \awaddr_reg_reg[4] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[4]),
        .Q(awaddr_reg[4]));
  FDCE \awaddr_reg_reg[5] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[5]),
        .Q(awaddr_reg[5]));
  FDCE \awaddr_reg_reg[6] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[6]),
        .Q(awaddr_reg[6]));
  FDCE \awaddr_reg_reg[7] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[7]),
        .Q(awaddr_reg[7]));
  FDCE \awaddr_reg_reg[8] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[8]),
        .Q(awaddr_reg[8]));
  FDCE \awaddr_reg_reg[9] 
       (.C(aclk),
        .CE(awaddr_reg_0),
        .CLR(aresetn_0),
        .D(S_AXI_AWADDR[9]),
        .Q(awaddr_reg[9]));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT3 #(
    .INIT(8'h20)) 
    mem_fin_i_1
       (.I0(aw_latched_reg_n_0),
        .I1(S_AXI_BVALID_reg_0),
        .I2(w_latched_reg_n_0),
        .O(mem_fin_i_1_n_0));
  FDCE mem_fin_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(mem_fin_i_1_n_0),
        .Q(init_write));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \mode[0]_i_1 
       (.I0(\wdata_reg_reg_n_0_[0] ),
        .I1(\mode[1]_i_2_n_0 ),
        .I2(mode[0]),
        .O(\mode[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \mode[1]_i_1 
       (.I0(\wdata_reg_reg_n_0_[1] ),
        .I1(\mode[1]_i_2_n_0 ),
        .I2(mode[1]),
        .O(\mode[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \mode[1]_i_2 
       (.I0(\mode[1]_i_3_n_0 ),
        .I1(\mode[1]_i_4_n_0 ),
        .I2(\mode[1]_i_5_n_0 ),
        .I3(\mode[1]_i_6_n_0 ),
        .I4(\mode[1]_i_7_n_0 ),
        .I5(\mode[1]_i_8_n_0 ),
        .O(\mode[1]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \mode[1]_i_3 
       (.I0(awaddr_reg[22]),
        .I1(awaddr_reg[23]),
        .I2(awaddr_reg[20]),
        .I3(awaddr_reg[21]),
        .I4(awaddr_reg[25]),
        .I5(awaddr_reg[24]),
        .O(\mode[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000000100000000)) 
    \mode[1]_i_4 
       (.I0(awaddr_reg[28]),
        .I1(awaddr_reg[29]),
        .I2(awaddr_reg[26]),
        .I3(awaddr_reg[27]),
        .I4(awaddr_reg[30]),
        .I5(awaddr_reg[31]),
        .O(\mode[1]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT5 #(
    .INIT(32'h00100000)) 
    \mode[1]_i_5 
       (.I0(awaddr_reg[1]),
        .I1(awaddr_reg[0]),
        .I2(aw_latched_reg_n_0),
        .I3(S_AXI_BVALID_reg_0),
        .I4(w_latched_reg_n_0),
        .O(\mode[1]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \mode[1]_i_6 
       (.I0(awaddr_reg[4]),
        .I1(awaddr_reg[5]),
        .I2(awaddr_reg[2]),
        .I3(awaddr_reg[3]),
        .I4(awaddr_reg[7]),
        .I5(awaddr_reg[6]),
        .O(\mode[1]_i_6_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \mode[1]_i_7 
       (.I0(awaddr_reg[16]),
        .I1(awaddr_reg[17]),
        .I2(awaddr_reg[14]),
        .I3(awaddr_reg[15]),
        .I4(awaddr_reg[19]),
        .I5(awaddr_reg[18]),
        .O(\mode[1]_i_7_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \mode[1]_i_8 
       (.I0(awaddr_reg[10]),
        .I1(awaddr_reg[11]),
        .I2(awaddr_reg[8]),
        .I3(awaddr_reg[9]),
        .I4(awaddr_reg[13]),
        .I5(awaddr_reg[12]),
        .O(\mode[1]_i_8_n_0 ));
  FDCE \mode_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\mode[0]_i_1_n_0 ),
        .Q(mode[0]));
  FDCE \mode_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\mode[1]_i_1_n_0 ),
        .Q(mode[1]));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT5 #(
    .INIT(32'hFF0F8888)) 
    w_latched_i_1
       (.I0(S_AXI_WREADY_reg_0),
        .I1(S_AXI_WVALID),
        .I2(aw_latched_reg_n_0),
        .I3(S_AXI_BVALID_reg_0),
        .I4(w_latched_reg_n_0),
        .O(w_latched_i_1_n_0));
  FDCE w_latched_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(w_latched_i_1_n_0),
        .Q(w_latched_reg_n_0));
  LUT4 #(
    .INIT(16'hBF80)) 
    \wdata_reg[0]_i_1 
       (.I0(S_AXI_WDATA[0]),
        .I1(S_AXI_WVALID),
        .I2(S_AXI_WREADY_reg_0),
        .I3(\wdata_reg_reg_n_0_[0] ),
        .O(\wdata_reg[0]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hBF80)) 
    \wdata_reg[1]_i_1 
       (.I0(S_AXI_WDATA[1]),
        .I1(S_AXI_WVALID),
        .I2(S_AXI_WREADY_reg_0),
        .I3(\wdata_reg_reg_n_0_[1] ),
        .O(\wdata_reg[1]_i_1_n_0 ));
  FDCE \wdata_reg_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\wdata_reg[0]_i_1_n_0 ),
        .Q(\wdata_reg_reg_n_0_[0] ));
  FDCE \wdata_reg_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(aresetn_0),
        .D(\wdata_reg[1]_i_1_n_0 ),
        .Q(\wdata_reg_reg_n_0_[1] ));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus
   (bram_web,
    bram_addrb,
    bram_dina,
    aclk,
    a_en_reg_0);
  output [0:0]bram_web;
  output [0:0]bram_addrb;
  output [0:0]bram_dina;
  input aclk;
  input a_en_reg_0;

  wire \FSM_sequential_state[0]_i_1_n_0 ;
  wire \FSM_sequential_state[1]_i_1_n_0 ;
  wire a_en_i_1_n_0;
  wire a_en_reg_0;
  wire a_we_i_1_n_0;
  wire aclk;
  wire \b_addr[0]_i_1_n_0 ;
  wire [0:0]bram_addrb;
  wire [0:0]bram_dina;
  wire [0:0]bram_web;
  wire [1:0]state;

  LUT2 #(
    .INIT(4'hB)) 
    \FSM_sequential_state[0]_i_1 
       (.I0(state[1]),
        .I1(state[0]),
        .O(\FSM_sequential_state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \FSM_sequential_state[1]_i_1 
       (.I0(state[0]),
        .I1(state[1]),
        .O(\FSM_sequential_state[1]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "IDLE:00,WRITE:01,READ:10,HOLD:11" *) 
  FDCE \FSM_sequential_state_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(a_en_reg_0),
        .D(\FSM_sequential_state[0]_i_1_n_0 ),
        .Q(state[0]));
  (* FSM_ENCODED_STATES = "IDLE:00,WRITE:01,READ:10,HOLD:11" *) 
  FDCE \FSM_sequential_state_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(a_en_reg_0),
        .D(\FSM_sequential_state[1]_i_1_n_0 ),
        .Q(state[1]));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT3 #(
    .INIT(8'hF1)) 
    a_en_i_1
       (.I0(state[1]),
        .I1(state[0]),
        .I2(bram_dina),
        .O(a_en_i_1_n_0));
  FDCE a_en_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(a_en_reg_0),
        .D(a_en_i_1_n_0),
        .Q(bram_dina));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT3 #(
    .INIT(8'h8B)) 
    a_we_i_1
       (.I0(bram_web),
        .I1(state[1]),
        .I2(state[0]),
        .O(a_we_i_1_n_0));
  FDCE a_we_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(a_en_reg_0),
        .D(a_we_i_1_n_0),
        .Q(bram_web));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \b_addr[0]_i_1 
       (.I0(bram_addrb),
        .I1(state[0]),
        .O(\b_addr[0]_i_1_n_0 ));
  FDCE \b_addr_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .CLR(a_en_reg_0),
        .D(\b_addr[0]_i_1_n_0 ),
        .Q(bram_addrb));
endmodule

(* CHECK_LICENSE_TYPE = "design_1_mem_controller_0_1,mem_controller,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "package_project" *) 
(* X_CORE_INFO = "mem_controller,Vivado 2018.3.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (aclk,
    aresetn,
    a_en,
    a_we,
    a_addr,
    a_din,
    a_dout,
    b_en,
    b_we,
    b_addr,
    b_din,
    b_dout,
    bram_ena,
    bram_wea,
    bram_addra,
    bram_dina,
    bram_douta,
    bram_rsta,
    bram_enb,
    bram_web,
    bram_addrb,
    bram_dinb,
    bram_doutb,
    bram_rstb,
    bram_en,
    bram_addr,
    bram_din,
    bram_we,
    bram_dout,
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
  wire [31:0]M_AXI_AWADDR;
  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_BREADY;
  wire [1:0]M_AXI_BRESP;
  wire M_AXI_BVALID;
  wire [31:0]M_AXI_WDATA;
  wire M_AXI_WREADY;
  wire [3:0]M_AXI_WSTRB;
  wire M_AXI_WVALID;
  wire [31:0]S_AXI_ARADDR;
  wire S_AXI_ARREADY;
  wire S_AXI_ARVALID;
  wire [31:0]S_AXI_AWADDR;
  wire S_AXI_AWREADY;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID;
  wire [1:0]\^S_AXI_RDATA ;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID;
  wire [31:0]S_AXI_WDATA;
  wire S_AXI_WREADY;
  wire S_AXI_WVALID;
  wire aclk;
  wire aresetn;
  wire [31:0]bram_addr;
  wire [0:0]\^bram_addrb ;
  wire [31:0]bram_din;
  wire [31:31]\^bram_dina ;
  wire [31:0]bram_dout;
  wire bram_en;
  wire [3:0]bram_we;
  wire [0:0]\^bram_web ;

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
  assign S_AXI_RDATA[1:0] = \^S_AXI_RDATA [1:0];
  assign S_AXI_RRESP[1] = \<const0> ;
  assign S_AXI_RRESP[0] = \<const0> ;
  assign bram_addra[3] = \<const0> ;
  assign bram_addra[2] = \<const0> ;
  assign bram_addra[1] = \<const0> ;
  assign bram_addra[0] = \<const0> ;
  assign bram_addrb[3] = \<const0> ;
  assign bram_addrb[2] = \<const0> ;
  assign bram_addrb[1] = \<const0> ;
  assign bram_addrb[0] = \^bram_addrb [0];
  assign bram_dina[31] = \^bram_dina [31];
  assign bram_dina[30] = \^bram_dina [31];
  assign bram_dina[29] = \<const0> ;
  assign bram_dina[28] = \^bram_dina [31];
  assign bram_dina[27] = \^bram_dina [31];
  assign bram_dina[26] = \^bram_dina [31];
  assign bram_dina[25] = \^bram_dina [31];
  assign bram_dina[24] = \<const0> ;
  assign bram_dina[23] = \^bram_dina [31];
  assign bram_dina[22] = \<const0> ;
  assign bram_dina[21] = \^bram_dina [31];
  assign bram_dina[20] = \<const0> ;
  assign bram_dina[19] = \^bram_dina [31];
  assign bram_dina[18] = \^bram_dina [31];
  assign bram_dina[17] = \<const0> ;
  assign bram_dina[16] = \^bram_dina [31];
  assign bram_dina[15] = \^bram_dina [31];
  assign bram_dina[14] = \<const0> ;
  assign bram_dina[13] = \^bram_dina [31];
  assign bram_dina[12] = \^bram_dina [31];
  assign bram_dina[11] = \^bram_dina [31];
  assign bram_dina[10] = \^bram_dina [31];
  assign bram_dina[9] = \^bram_dina [31];
  assign bram_dina[8] = \<const0> ;
  assign bram_dina[7] = \^bram_dina [31];
  assign bram_dina[6] = \^bram_dina [31];
  assign bram_dina[5] = \^bram_dina [31];
  assign bram_dina[4] = \<const0> ;
  assign bram_dina[3] = \^bram_dina [31];
  assign bram_dina[2] = \^bram_dina [31];
  assign bram_dina[1] = \^bram_dina [31];
  assign bram_dina[0] = \^bram_dina [31];
  assign bram_dinb[31] = \^bram_dina [31];
  assign bram_dinb[30] = \^bram_dina [31];
  assign bram_dinb[29] = \<const0> ;
  assign bram_dinb[28] = \^bram_dina [31];
  assign bram_dinb[27] = \^bram_dina [31];
  assign bram_dinb[26] = \^bram_dina [31];
  assign bram_dinb[25] = \^bram_dina [31];
  assign bram_dinb[24] = \<const0> ;
  assign bram_dinb[23] = \^bram_dina [31];
  assign bram_dinb[22] = \<const0> ;
  assign bram_dinb[21] = \^bram_dina [31];
  assign bram_dinb[20] = \<const0> ;
  assign bram_dinb[19] = \^bram_dina [31];
  assign bram_dinb[18] = \^bram_dina [31];
  assign bram_dinb[17] = \<const0> ;
  assign bram_dinb[16] = \^bram_dina [31];
  assign bram_dinb[15] = \^bram_dina [31];
  assign bram_dinb[14] = \<const0> ;
  assign bram_dinb[13] = \^bram_dina [31];
  assign bram_dinb[12] = \^bram_dina [31];
  assign bram_dinb[11] = \^bram_dina [31];
  assign bram_dinb[10] = \^bram_dina [31];
  assign bram_dinb[9] = \^bram_dina [31];
  assign bram_dinb[8] = \<const0> ;
  assign bram_dinb[7] = \^bram_dina [31];
  assign bram_dinb[6] = \^bram_dina [31];
  assign bram_dinb[5] = \^bram_dina [31];
  assign bram_dinb[4] = \<const0> ;
  assign bram_dinb[3] = \^bram_dina [31];
  assign bram_dinb[2] = \^bram_dina [31];
  assign bram_dinb[1] = \^bram_dina [31];
  assign bram_dinb[0] = \^bram_dina [31];
  assign bram_ena = \^bram_dina [31];
  assign bram_enb = \^bram_dina [31];
  assign bram_rsta = aresetn;
  assign bram_rstb = aresetn;
  assign bram_wea[3] = \^bram_web [0];
  assign bram_wea[2] = \^bram_web [0];
  assign bram_wea[1] = \^bram_web [0];
  assign bram_wea[0] = \^bram_web [0];
  assign bram_web[3] = \^bram_web [0];
  assign bram_web[2] = \^bram_web [0];
  assign bram_web[1] = \^bram_web [0];
  assign bram_web[0] = \^bram_web [0];
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller inst
       (.M_AXI_AWADDR(M_AXI_AWADDR),
        .M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_AWVALID(M_AXI_AWVALID),
        .M_AXI_BREADY(M_AXI_BREADY),
        .M_AXI_BRESP(M_AXI_BRESP),
        .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_WDATA(M_AXI_WDATA),
        .M_AXI_WREADY(M_AXI_WREADY),
        .M_AXI_WSTRB(M_AXI_WSTRB),
        .M_AXI_WVALID(M_AXI_WVALID),
        .S_AXI_ARADDR(S_AXI_ARADDR),
        .S_AXI_ARREADY_reg(S_AXI_ARREADY),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_AWADDR(S_AXI_AWADDR),
        .S_AXI_AWREADY_reg(S_AXI_AWREADY),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_BVALID_reg(S_AXI_BVALID),
        .S_AXI_RDATA(\^S_AXI_RDATA ),
        .S_AXI_RREADY(S_AXI_RREADY),
        .S_AXI_RVALID_reg(S_AXI_RVALID),
        .S_AXI_WDATA(S_AXI_WDATA[1:0]),
        .S_AXI_WREADY_reg(S_AXI_WREADY),
        .S_AXI_WVALID(S_AXI_WVALID),
        .aclk(aclk),
        .aresetn(aresetn),
        .bram_addr(bram_addr),
        .bram_addrb(\^bram_addrb ),
        .bram_din(bram_din),
        .bram_dina(\^bram_dina ),
        .bram_dout(bram_dout),
        .bram_en(bram_en),
        .bram_we(bram_we),
        .bram_web(\^bram_web ));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller
   (bram_addrb,
    S_AXI_ARREADY_reg,
    bram_dout,
    M_AXI_AWADDR,
    M_AXI_AWVALID,
    M_AXI_WDATA,
    M_AXI_WSTRB,
    M_AXI_WVALID,
    M_AXI_BREADY,
    S_AXI_WREADY_reg,
    S_AXI_RDATA,
    S_AXI_BVALID_reg,
    S_AXI_AWREADY_reg,
    S_AXI_RVALID_reg,
    bram_dina,
    bram_web,
    S_AXI_ARVALID,
    aclk,
    bram_en,
    bram_addr,
    bram_din,
    bram_we,
    aresetn,
    M_AXI_AWREADY,
    M_AXI_WREADY,
    M_AXI_BRESP,
    M_AXI_BVALID,
    S_AXI_AWADDR,
    S_AXI_WDATA,
    S_AXI_WVALID,
    S_AXI_ARADDR,
    S_AXI_AWVALID,
    S_AXI_BREADY,
    S_AXI_RREADY);
  output [0:0]bram_addrb;
  output S_AXI_ARREADY_reg;
  output [31:0]bram_dout;
  output [31:0]M_AXI_AWADDR;
  output M_AXI_AWVALID;
  output [31:0]M_AXI_WDATA;
  output [3:0]M_AXI_WSTRB;
  output M_AXI_WVALID;
  output M_AXI_BREADY;
  output S_AXI_WREADY_reg;
  output [1:0]S_AXI_RDATA;
  output S_AXI_BVALID_reg;
  output S_AXI_AWREADY_reg;
  output S_AXI_RVALID_reg;
  output [0:0]bram_dina;
  output [0:0]bram_web;
  input S_AXI_ARVALID;
  input aclk;
  input bram_en;
  input [31:0]bram_addr;
  input [31:0]bram_din;
  input [3:0]bram_we;
  input aresetn;
  input M_AXI_AWREADY;
  input M_AXI_WREADY;
  input [1:0]M_AXI_BRESP;
  input M_AXI_BVALID;
  input [31:0]S_AXI_AWADDR;
  input [1:0]S_AXI_WDATA;
  input S_AXI_WVALID;
  input [31:0]S_AXI_ARADDR;
  input S_AXI_AWVALID;
  input S_AXI_BREADY;
  input S_AXI_RREADY;

  wire [31:0]M_AXI_AWADDR;
  wire M_AXI_AWREADY;
  wire M_AXI_AWVALID;
  wire M_AXI_BREADY;
  wire [1:0]M_AXI_BRESP;
  wire M_AXI_BVALID;
  wire [31:0]M_AXI_WDATA;
  wire M_AXI_WREADY;
  wire [3:0]M_AXI_WSTRB;
  wire M_AXI_WVALID;
  wire [31:0]S_AXI_ARADDR;
  wire S_AXI_ARREADY_reg;
  wire S_AXI_ARVALID;
  wire [31:0]S_AXI_AWADDR;
  wire S_AXI_AWREADY_reg;
  wire S_AXI_AWVALID;
  wire S_AXI_BREADY;
  wire S_AXI_BVALID_reg;
  wire [1:0]S_AXI_RDATA;
  wire S_AXI_RREADY;
  wire S_AXI_RVALID_reg;
  wire [1:0]S_AXI_WDATA;
  wire S_AXI_WREADY_reg;
  wire S_AXI_WVALID;
  wire aclk;
  wire aresetn;
  wire [31:0]bram_addr;
  wire [0:0]bram_addrb;
  wire [31:0]bram_din;
  wire [0:0]bram_dina;
  wire [31:0]bram_dout;
  wire bram_en;
  wire [3:0]bram_we;
  wire [0:0]bram_web;
  wire init_out;
  wire slave1_n_1;
  wire NLW_master1_bram_ready_UNCONNECTED;
  wire [1:0]NLW_master1_state_led_UNCONNECTED;

  (* ST_AWBRAM = "2'b11" *) 
  (* ST_AW_W = "2'b00" *) 
  (* ST_B = "2'b01" *) 
  (* ST_DONE = "2'b10" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master master1
       (.M_AXI_AWADDR(M_AXI_AWADDR),
        .M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_AWVALID(M_AXI_AWVALID),
        .M_AXI_BREADY(M_AXI_BREADY),
        .M_AXI_BRESP(M_AXI_BRESP),
        .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_WDATA(M_AXI_WDATA),
        .M_AXI_WREADY(M_AXI_WREADY),
        .M_AXI_WSTRB(M_AXI_WSTRB),
        .M_AXI_WVALID(M_AXI_WVALID),
        .aclk(aclk),
        .addr_in({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .aresetn(aresetn),
        .bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_dout(bram_dout),
        .bram_en(bram_en),
        .bram_ready(NLW_master1_bram_ready_UNCONNECTED),
        .bram_we(bram_we),
        .init_write(init_out),
        .state_led(NLW_master1_state_led_UNCONNECTED[1:0]),
        .value_out({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave slave1
       (.S_AXI_ARADDR(S_AXI_ARADDR),
        .S_AXI_ARREADY_reg_0(S_AXI_ARREADY_reg),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_AWADDR(S_AXI_AWADDR),
        .S_AXI_AWREADY_reg_0(S_AXI_AWREADY_reg),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_BVALID_reg_0(S_AXI_BVALID_reg),
        .S_AXI_RDATA(S_AXI_RDATA),
        .S_AXI_RREADY(S_AXI_RREADY),
        .S_AXI_RVALID_reg_0(S_AXI_RVALID_reg),
        .S_AXI_WDATA(S_AXI_WDATA),
        .S_AXI_WREADY_reg_0(S_AXI_WREADY_reg),
        .S_AXI_WVALID(S_AXI_WVALID),
        .aclk(aclk),
        .aresetn(aresetn),
        .aresetn_0(slave1_n_1),
        .init_write(init_out));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus stimmy
       (.a_en_reg_0(slave1_n_1),
        .aclk(aclk),
        .bram_addrb(bram_addrb),
        .bram_dina(bram_dina),
        .bram_web(bram_web));
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
