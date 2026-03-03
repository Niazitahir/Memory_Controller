`timescale 1ns / 1ps

module lsr_axi532
(
  input aclk,
  input aresetn,

  // AXI-Lite slave interface
  input [31:0]  S_AXI_AWADDR,
  input         S_AXI_AWVALID,
  output        S_AXI_AWREADY,

  input [31:0]  S_AXI_WDATA,
  input [3:0]   S_AXI_WSTRB,
  input         S_AXI_WVALID,
  output        S_AXI_WREADY,

  output [1:0]  S_AXI_BRESP,
  output        S_AXI_BVALID,
  input         S_AXI_BREADY,

  input [31:0]  S_AXI_ARADDR,
  input         S_AXI_ARVALID,
  output        S_AXI_ARREADY,

  output [31:0] S_AXI_RDATA,
  output [1:0]  S_AXI_RRESP,
  output        S_AXI_RVALID,
  input         S_AXI_RREADY,

  // AXI-Lite master interface
  output [31:0] M_AXI_AWADDR,
  output        M_AXI_AWVALID,
  input         M_AXI_AWREADY,

  output [31:0] M_AXI_WDATA,
  output [3:0]  M_AXI_WSTRB,
  output        M_AXI_WVALID,
  input         M_AXI_WREADY,

  input [1:0]   M_AXI_BRESP,
  input         M_AXI_BVALID,
  output        M_AXI_BREADY
);
    wire [31:0] val_out;
    wire init_out;
    wire [31:0] lsr_in;
    wire [31:0] lsr_n;
    wire [31:0] addr;
axi_lite_slave slave1 (
  .clk(aclk),
  .resetn(aresetn),
  .S_AXI_AWADDR(S_AXI_AWADDR),
  .S_AXI_AWVALID(S_AXI_AWVALID),
  .S_AXI_AWREADY(S_AXI_AWREADY),
  
  .S_AXI_WDATA(S_AXI_WDATA),
  .S_AXI_WSTRB(S_AXI_WSTRB),
  .S_AXI_WVALID(S_AXI_WVALID),
  .S_AXI_WREADY(S_AXI_WREADY),
  
  .S_AXI_BRESP(S_AXI_BRESP),
  .S_AXI_BVALID(S_AXI_BVALID),
  .S_AXI_BREADY(S_AXI_BREADY),
  
  .S_AXI_ARADDR(S_AXI_ARADDR),
  .S_AXI_ARVALID(S_AXI_ARVALID),
  .S_AXI_ARREADY(S_AXI_ARREADY),
  
  .S_AXI_RDATA(S_AXI_RDATA),
  .S_AXI_RRESP(S_AXI_RRESP),
  .S_AXI_RVALID(S_AXI_RVALID),
  .S_AXI_RREADY(S_AXI_RREADY),
  .init_write(init_out),
  .val_out(lsr_in),
  .n_lsr(lsr_n),
  .addr_out(addr)
);

axi_lite_master master1 (
    .aclk (aclk),
    .aresetn (aresetn), 
    .init_write(init_out),
    .value_out(val_out),
    .M_AXI_AWADDR (M_AXI_AWADDR),
    .M_AXI_AWVALID(M_AXI_AWVALID),
    .M_AXI_AWREADY(M_AXI_AWREADY),
    
    .M_AXI_WDATA(M_AXI_WDATA),
    .M_AXI_WSTRB(M_AXI_WSTRB),
    .M_AXI_WVALID(M_AXI_WVALID),
    .M_AXI_WREADY(M_AXI_WREADY),
    
    .M_AXI_BRESP(M_AXI_BRESP),
    .M_AXI_BVALID(M_AXI_BVALID),
    .M_AXI_BREADY(M_AXI_BREADY),
    .addr_in(addr)
);
lsr532 lsr(
    .n(lsr_n),
    .value_in(lsr_in),
    .value_out(val_out)
);
endmodule

//cause im pedantic and love complicating things
//note, only a single 32bit register for reads and writes
//writing more than one thing will overwrite it. 
module axi_lite_slave (
  input         clk,
  input         resetn,
  input  [31:0] S_AXI_AWADDR,
  input         S_AXI_AWVALID,
  output reg    S_AXI_AWREADY,

  input  [31:0] S_AXI_WDATA,
  input  [3:0]  S_AXI_WSTRB,
  input         S_AXI_WVALID,
  output reg    S_AXI_WREADY,

  output reg [1:0] S_AXI_BRESP,
  output reg       S_AXI_BVALID,
  input            S_AXI_BREADY,

  input  [31:0] S_AXI_ARADDR,
  input         S_AXI_ARVALID,
  output reg    S_AXI_ARREADY,

  output reg [31:0] S_AXI_RDATA,
  output reg [1:0]  S_AXI_RRESP,
  output reg        S_AXI_RVALID,
  input             S_AXI_RREADY,
  output            init_write,
  output      [31:0] val_out,
  output    [31:0] n_lsr, 
  output [31:0] addr_out
);

  reg [4:0] n_out;//XXX8
  reg [31:0] value_in; //XXX0
  reg [31:0] output_addr;//XXX4
  reg mem_fin;
  reg [31:0] awaddr_latched;
  reg [31:0] araddr_latched;
  integer mem_fin_ctr;
  assign val_out = value_in;
  assign init_write = mem_fin;
  assign n_lsr = n_out;
  assign addr_out = output_addr;
  //write control
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        S_AXI_AWREADY <= 0;
        mem_fin_ctr <= 0;
    end 
    else begin
            if (!S_AXI_WREADY || !S_AXI_RREADY) begin
            S_AXI_AWREADY  <= 1'b0;
            end
            else begin
            S_AXI_AWREADY  <= 1'b1;          // accept address
            awaddr_latched <= S_AXI_AWADDR;
            end
        end
  end
  //actually writing to mem
  always @(posedge clk) begin
    if (!resetn) begin
        S_AXI_WREADY <= 1'b0;
        mem_fin <= 1'b0;
    end else begin
    //always set mem_fin to 0 unless written to i guess
        S_AXI_WREADY <= 1'b1;           // accept data
        if (mem_fin == 1'b1) begin
            mem_fin <= 1'b0;
        end

        if (S_AXI_WREADY && S_AXI_WVALID) begin
            S_AXI_WREADY <= 1'b0;
            case (awaddr_latched)
                32'h80000000: begin
                    if (S_AXI_WSTRB[0]) value_in[7:0]   <= S_AXI_WDATA[7:0];
                    if (S_AXI_WSTRB[1]) value_in[15:8]  <= S_AXI_WDATA[15:8];
                    if (S_AXI_WSTRB[2]) value_in[23:16] <= S_AXI_WDATA[23:16];
                    if (S_AXI_WSTRB[3]) value_in[31:24] <= S_AXI_WDATA[31:24];
                    
                end
                32'h80000004: begin
                    output_addr <= S_AXI_WDATA[31:0];
                end
                32'h80000008: begin
                    n_out <= S_AXI_WDATA[4:0];
                end
                32'h8000000C: begin
                    mem_fin <= S_AXI_WDATA[0];
                end
                default: begin
                    
                end

            endcase

        end
    end
  end
  //responding to that write
reg aw_hs_done;
reg w_hs_done;

always @(posedge clk) begin
    if (!resetn) begin
        aw_hs_done <= 0;
        w_hs_done  <= 0;
        S_AXI_BVALID <= 0;
        S_AXI_BRESP  <= 2'b00;
    end else begin
        // Detect AW handshake
        if (S_AXI_AWREADY && S_AXI_AWVALID)
            aw_hs_done <= 1;

        // Detect W handshake
        if (S_AXI_WREADY && S_AXI_WVALID)
            w_hs_done <= 1;

        // When both are done, assert BVALID
        if (aw_hs_done && w_hs_done)
            S_AXI_BVALID <= 1;

        // When master accepts the response, clear everything
        if (S_AXI_BREADY && S_AXI_BVALID) begin
            S_AXI_BVALID <= 0;
            aw_hs_done <= 0;
            w_hs_done <= 0;
        end
    end
end

  //read pretty much the same
always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      S_AXI_ARREADY   <= 1'b0;
      araddr_latched  <= 32'd0;
    end else begin
      if (!S_AXI_WREADY || !S_AXI_RREADY) begin
        S_AXI_ARREADY  <= 1'b1;         // accept read address
        araddr_latched <= S_AXI_ARADDR;
      end else begin
        S_AXI_ARREADY  <= 1'b0;
      end
    end
  end

 always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      S_AXI_RVALID <= 1'b0;
      S_AXI_RRESP  <= 2'b00; // OKAY
      S_AXI_RDATA  <= 32'd0;
    end else begin
      if (!S_AXI_RVALID && S_AXI_ARREADY && S_AXI_ARVALID) begin
        case (araddr_latched)
          32'h80000000: begin 
          S_AXI_RDATA <= value_in;
          S_AXI_RRESP<= 2'b00; 
          end
          32'h80000004: begin 
          S_AXI_RDATA <= output_addr; 
          S_AXI_RRESP<= 2'b00;
          end
          32'h80000008: begin 
          S_AXI_RDATA <= n_out;
          S_AXI_RRESP<= 2'b00;
          end
          default: begin
          S_AXI_RDATA <= 32'hBEEFBEEF;
          S_AXI_RRESP<= 2'b10;
          end
        endcase
        S_AXI_RVALID <= 1'b1;
      end else if (S_AXI_RVALID && S_AXI_RREADY) begin
        S_AXI_RVALID <= 1'b0;
      end
    end
  end

endmodule

module axi_lite_master (
    input         aclk,
    input         aresetn,
    input         init_write,
    output reg [31:0] M_AXI_AWADDR,
    output reg        M_AXI_AWVALID,
    input             M_AXI_AWREADY,

    output reg [31:0] M_AXI_WDATA,
    output reg [3:0]  M_AXI_WSTRB,
    output reg        M_AXI_WVALID,
    input             M_AXI_WREADY,

    input      [1:0]  M_AXI_BRESP,
    input             M_AXI_BVALID,
    output reg        M_AXI_BREADY,
    
    input      [31:0] value_out,
    input       [31:0] addr_in
);
    reg [1:0] state;
    reg [31:0] val_save;
    reg [31:0] latch_addr;
    //FSM Three stages
    localparam ST_AW_W = 2'b00;
    localparam ST_B    = 2'b01;
    localparam ST_DONE = 2'b10;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            M_AXI_AWADDR  <= 32'h00000000;
            M_AXI_AWVALID <= 1'b0;
            M_AXI_WDATA   <= 32'h00000000;
            M_AXI_WSTRB   <= 4'hF;
            M_AXI_WVALID  <= 1'b0;
            M_AXI_BREADY  <= 1'b0;
            latch_addr <= 32'h00000000;
            state         <= ST_DONE;
        end else begin
            case (state)

                ST_AW_W: begin
                latch_addr <= addr_in;
                    val_save = value_out;    
                    // drive address + data and hold VALID until handshake
                    M_AXI_AWADDR  <= latch_addr;   // matches mem[0]
                    M_AXI_WDATA   <= val_save;
                    M_AXI_WSTRB   <= 4'hF;

                    if (!M_AXI_AWVALID)
                        M_AXI_AWVALID <= 1'b1;
                    if (!M_AXI_WVALID)
                        M_AXI_WVALID  <= 1'b1;

                    if (M_AXI_AWVALID && M_AXI_AWREADY)
                        M_AXI_AWVALID <= 1'b0;
                    if (M_AXI_WVALID && M_AXI_WREADY)
                        M_AXI_WVALID  <= 1'b0;

                    if ((M_AXI_AWVALID && M_AXI_AWREADY) &&
                        (M_AXI_WVALID  && M_AXI_WREADY)) begin
                        M_AXI_BREADY <= 1'b1;
                        state        <= ST_B;
                    end
                end

                ST_B: begin
                    if (M_AXI_BVALID) begin
                        M_AXI_BREADY <= 1'b0;
                        state        <= ST_DONE;
                    end
                end

                ST_DONE: begin
                if (init_write)
                    state <= ST_AW_W;
                end

            endcase
        end
    end

endmodule

module lsr532
(
  input [4:0] 	n,
  input [31:0]  value_in,
  output [31:0] value_out
);
 // n[0] = shift by one, n[1] = shift by 2 etc. 
  wire [31:0] s1, s2, s4, s8, s16;

    // Rotate by 1
    assign s1  = n[0] ? {value_in[30:0],  value_in[31]}  : value_in;

    // Rotate by 2
    assign s2  = n[1] ? {s1[29:0],  s1[31:30]} : s1;

    // Rotate by 4
    assign s4  = n[2] ? {s2[27:0],  s2[31:28]} : s2;

    // Rotate by 8
    assign s8  = n[3] ? {s4[23:0],  s4[31:24]} : s4;

    // Rotate by 16
    assign s16 = n[4] ? {s8[15:0],  s8[31:16]} : s8;

    assign value_out = s16;
	// Your Code Here

endmodule