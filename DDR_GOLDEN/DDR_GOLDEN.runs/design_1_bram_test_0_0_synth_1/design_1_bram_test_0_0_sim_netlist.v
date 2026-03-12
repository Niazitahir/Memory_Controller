// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
// Date        : Tue Mar 10 14:04:57 2026
// Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_bram_test_0_0_sim_netlist.v
// Design      : design_1_bram_test_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_test
   (clk,
    reset_n,
    bram_en,
    bram_addr,
    bram_din,
    bram_we,
    bram_dout,
    bram_ready,
    tester,
    test_pass);
  input clk;
  input reset_n;
  output bram_en;
  (* mark_debug = "true" *) output [31:0]bram_addr;
  output [31:0]bram_din;
  output [3:0]bram_we;
  input [31:0]bram_dout;
  input bram_ready;
  input tester;
  output test_pass;

  wire \<const0> ;
  (* MARK_DEBUG *) wire [31:0]bram_addr;
  wire [31:31]\^bram_din ;
  wire bram_en_i_1_n_0;
  wire clk;
  wire reset_n;
  wire test_pass;
  wire tester;

  assign bram_din[31] = \^bram_din [31];
  assign bram_din[30] = \^bram_din [31];
  assign bram_din[29] = \<const0> ;
  assign bram_din[28] = \^bram_din [31];
  assign bram_din[27] = \^bram_din [31];
  assign bram_din[26] = \^bram_din [31];
  assign bram_din[25] = \^bram_din [31];
  assign bram_din[24] = \<const0> ;
  assign bram_din[23] = \^bram_din [31];
  assign bram_din[22] = \<const0> ;
  assign bram_din[21] = \^bram_din [31];
  assign bram_din[20] = \<const0> ;
  assign bram_din[19] = \^bram_din [31];
  assign bram_din[18] = \^bram_din [31];
  assign bram_din[17] = \<const0> ;
  assign bram_din[16] = \^bram_din [31];
  assign bram_din[15] = \^bram_din [31];
  assign bram_din[14] = \<const0> ;
  assign bram_din[13] = \^bram_din [31];
  assign bram_din[12] = \^bram_din [31];
  assign bram_din[11] = \^bram_din [31];
  assign bram_din[10] = \^bram_din [31];
  assign bram_din[9] = \^bram_din [31];
  assign bram_din[8] = \<const0> ;
  assign bram_din[7] = \^bram_din [31];
  assign bram_din[6] = \^bram_din [31];
  assign bram_din[5] = \^bram_din [31];
  assign bram_din[4] = \<const0> ;
  assign bram_din[3] = \^bram_din [31];
  assign bram_din[2] = \^bram_din [31];
  assign bram_din[1] = \^bram_din [31];
  assign bram_din[0] = \^bram_din [31];
  assign bram_en = \^bram_din [31];
  assign bram_we[3] = \^bram_din [31];
  assign bram_we[2] = \^bram_din [31];
  assign bram_we[1] = \^bram_din [31];
  assign bram_we[0] = \^bram_din [31];
  GND GND
       (.G(\<const0> ));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[0]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[10] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[10]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[11] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[11]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[12] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[12]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[13] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[13]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[14] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[14]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[15] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[15]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[16] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[16]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[17] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[17]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[18] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[18]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[19] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[19]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[1]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[20] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[20]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[21] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[21]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[22] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[22]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[23] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[23]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[24] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[24]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[25] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[25]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[26] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[26]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[27] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[27]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[28] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[28]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[29] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[29]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[2]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[30] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(test_pass),
        .Q(bram_addr[30]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[31] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(test_pass),
        .Q(bram_addr[31]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[3] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[3]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[4] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[4]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[5] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[5]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[6] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[6]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[7] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[7]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[8] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[8]));
  (* KEEP = "yes" *) 
  FDCE \bram_addr_reg[9] 
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(1'b0),
        .Q(bram_addr[9]));
  LUT1 #(
    .INIT(2'h1)) 
    bram_en_i_1
       (.I0(reset_n),
        .O(bram_en_i_1_n_0));
  FDPE bram_en_reg
       (.C(clk),
        .CE(1'b1),
        .D(test_pass),
        .PRE(bram_en_i_1_n_0),
        .Q(\^bram_din ));
  FDCE tester_latch_reg
       (.C(clk),
        .CE(1'b1),
        .CLR(bram_en_i_1_n_0),
        .D(tester),
        .Q(test_pass));
endmodule

(* CHECK_LICENSE_TYPE = "design_1_bram_test_0_0,bram_test,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "bram_test,Vivado 2018.3.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    reset_n,
    bram_en,
    bram_addr,
    bram_din,
    bram_we,
    bram_dout,
    bram_ready,
    tester,
    test_pass);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clk, FREQ_HZ 81247969, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, INSERT_VIP 0" *) input clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  output bram_en;
  output [31:0]bram_addr;
  output [31:0]bram_din;
  output [3:0]bram_we;
  input [31:0]bram_dout;
  input bram_ready;
  input tester;
  output test_pass;

  wire [31:0]bram_addr;
  wire [31:0]bram_din;
  wire [31:0]bram_dout;
  wire bram_en;
  wire bram_ready;
  wire [3:0]bram_we;
  wire clk;
  wire reset_n;
  wire test_pass;
  wire tester;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_test inst
       (.bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_dout(bram_dout),
        .bram_en(bram_en),
        .bram_ready(bram_ready),
        .bram_we(bram_we),
        .clk(clk),
        .reset_n(reset_n),
        .test_pass(test_pass),
        .tester(tester));
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
