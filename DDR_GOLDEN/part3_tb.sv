`timescale 1ns / 1ps

import axi_vip_pkg::*;
import design_1_axi_vip_1_0_pkg::*;


// test module to drive the AXI VIP
module axi_lite_stimulus(

    input logic aclk,
    input logic aresetn
);
    

// AXI VIP master agent handle
    design_1_axi_vip_1_0_mst_t master_agent;
    reg [31:0] write_data = 32'h0123456;
    reg [31:0] write_data2 = 32'hDEADBEEF;
    reg [31:0] n_val = 32'h00000008;
    reg [31:0] n_val2 = 32'h0000000F;
    reg [31:0] read_data;

    reg [1:0] resp;
initial begin
        
    // Wait for reset to deassert
    wait (part3_tb.aresetn == 1);

    // Create and start the AXI VIP master
    master_agent = new("master_agent", part3_tb.DUT.design_1_i.axi_vip_1.inst.IF);
    master_agent.start_master();

    $display("Stimulus started at time %0t", $time);

    // ------------------------------------------------------------
    // 1. Set mode = 2'b10 (word writes)
    // ------------------------------------------------------------
    master_agent.AXI4LITE_WRITE_BURST(
        .addr(32'h80000000),
        .data(32'h00000002),   // mode = 2'b10
        .prot(3'b000),
        .resp(resp)
    );

    // ------------------------------------------------------------
    // 2. Fill COLUMN 0 with DEADBEEF (3840 bits = 120 words)
    // ------------------------------------------------------------
    for (int i = 0; i < 119; i++) begin
        master_agent.AXI4LITE_WRITE_BURST(
            .addr(32'h80000004 + i*4),   // column 0 base + offset
            .data(32'hDEADBEEF),
            .prot(3'b000),
            .resp(resp)
        );
    end

    $display("Finished writing column 0");

    // ------------------------------------------------------------
    // 3. Optional: Read back a few words to verify
    // ------------------------------------------------------------
    for (int i = 0; i < 4; i++) begin
        master_agent.AXI4LITE_READ_BURST(
            .addr(32'h80000000 + i*4),
            .data(read_data),
            .prot(3'b000),
            .resp(resp)
        );
        $display("Readback[%0d] = %h", i, read_data);
    end

end


endmodule 


// testbench entry point
module part3_tb();

	reg aclk;
	reg aresetn;

	// instantiate the "design under test" module
	design_1_wrapper DUT(
		.clk(aclk),
		.resetn(aresetn)
		);

	// clock generator (100MHz)
	initial
	begin
		aclk = 0;
		forever
			#5ns aclk = ~aclk;
	end

	// start the testbench by resetting the system for 5 cycles
	initial
	begin
		aresetn = 0;
		repeat(5) @(posedge aclk);
		forever
			@(posedge aclk) aresetn = 1;
	end

	// instantiate instance of axi_lite_stimulus into the tb
	axi_lite_stimulus mst(
    .aclk(aclk),
    .aresetn(aresetn)
    );


endmodule

