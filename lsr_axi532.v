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

axi_lite_slave slave1 (
  clk(aclk),
  resetn(aresetn),
  S_AXI_AWADDR(S_AXI_AWADDR),
  S_AXI_AWVALID(S_AXI_AWVALID),
  S_AXI_AWREADY(S_AXI_AWREADY),

  S_AXI_WDATA(S_AXI_WDATA),
  S_AXI_WSTRB(S_AXI_WSTRB),
  S_AXI_WVALID(S_AXI_WVALID),
  S_AXI_WREADY(S_AXI_WREADY),

  S_AXI_BRESP(S_AXI_BRESP),
  S_AXI_BVALID(S_AXI_BVALID),
  S_AXI_BREADY(S_AXI_BREADY),

  S_AXI_ARADDR(S_AXI_ARADDR),
  S_AXI_ARVALID(S_AXI_ARVALID),
  S_AXI_ARREADY(S_AXI_ARREADY),

  S_AXI_RDATA(S_AXI_RDATA),
  S_AXI_RRESP(S_AXI_RRESP),
  S_AXI_RVALID(S_AXI_RVALID),
  S_AXI_RREADY(S_AXI_RREADY)
);

axi_lite_master master1 (
    aclk (aclk),
    aresetn (aresetn), 

    M_AXI_AWADDR (M_AXI_AWADDR),
    M_AXI_AWVALID(M_AXI_AWVALID),
    M_AXI_AWREADY(M_AXI_AWREADY),

    M_AXI_WDATA(M_AXI_WDATA),
    M_AXI_WSTRB(M_AXI_WSTRB),
    M_AXI_WVALID(M_AXI_WVALID),
    M_AXI_WREADY(M_AXI_WREADY),

    M_AXI_BRESP(M_AXI_BRESP),
    M_AXI_BVALID(M_AXI_BVALID),
    M_AXI_BREADY(M_AXI_BREADY)
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
  input             S_AXI_RREADY
);
  reg [31:0] reg0;  
  reg [31:0] awaddr_latched;
  reg [31:0] araddr_latched;

  //write control
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        S_AXI_AWREADY <= 0;
    end 
    else begin
        if (!S_AXI_AWREADY && S_AXI_AWVALID)
            S_AXI_AWREADY <= 1;       // accept address
        else
            S_AXI_AWREADY <= 0;

        if (S_AXI_AWREADY && S_AXI_AWVALID)
            awaddr_latched <= S_AXI_AWADDR;
    end
  end
  //actually writing to mem
  always @(posedge clk) begin
    if (!resetn) begin
        S_AXI_WREADY <= 0;
    end else begin
        if (!S_AXI_WREADY && S_AXI_WVALID)
            S_AXI_WREADY <= 1;        // accept data
        else
            S_AXI_WREADY <= 0;

        if (S_AXI_WREADY && S_AXI_WVALID) begin
            case (awaddr_latched[3:2])
                2'b00: begin
                    if (S_AXI_WSTRB[0]) reg0[7:0]   <= S_AXI_WDATA[7:0];
                    if (S_AXI_WSTRB[1]) reg0[15:8]  <= S_AXI_WDATA[15:8];
                    if (S_AXI_WSTRB[2]) reg0[23:16] <= S_AXI_WDATA[23:16];
                    if (S_AXI_WSTRB[3]) reg0[31:24] <= S_AXI_WDATA[31:24];
                end
            endcase
        end
    end
  end
  //responding to that write
  always @(posedge clk) begin
    if (!resetn) begin
        S_AXI_BVALID <= 0;
        S_AXI_BRESP  <= 2'b00; // Lets do this
    end else begin
        if (S_AXI_WREADY && S_AXI_WVALID && S_AXI_AWREADY && S_AXI_AWVALID)
            S_AXI_BVALID <= 1;        // write complete
        else if (S_AXI_BREADY)
            S_AXI_BVALID <= 0;
    end
  end
  //read pretty much the same
  always @(posedge clk) begin
    if (!resetn) begin
        S_AXI_ARREADY <= 0;
    end else begin
        if (!S_AXI_ARREADY && S_AXI_ARVALID)
            S_AXI_ARREADY <= 1;       // accept read address
        else
            S_AXI_ARREADY <= 0;

        if (S_AXI_ARREADY && S_AXI_ARVALID)
            araddr_latched <= S_AXI_ARADDR;
    end
  end
  always @(posedge clk) begin
    if (!resetn) begin
        S_AXI_RVALID <= 0;
        S_AXI_RRESP  <= 2'b00; // OKAY
        S_AXI_RDATA  <= 0;
    end else begin
        if (S_AXI_ARREADY && S_AXI_ARVALID) begin
            case (araddr_latched[3:2])
                2'b00: S_AXI_RDATA <= reg0;
                default: S_AXI_RDATA <= 32'hDEADBEEF;
            endcase
            S_AXI_RVALID <= 1;
        end else if (S_AXI_RREADY)
            S_AXI_RVALID <= 0;
    end
  end
endmodule

module axi_lite_master (
    input         aclk,
    input         aresetn,

    output reg [31:0] M_AXI_AWADDR,
    output reg        M_AXI_AWVALID,
    input             M_AXI_AWREADY,

    output reg [31:0] M_AXI_WDATA,
    output reg [3:0]  M_AXI_WSTRB,
    output reg        M_AXI_WVALID,
    input             M_AXI_WREADY,

    input      [1:0]  M_AXI_BRESP,
    input             M_AXI_BVALID,
    output reg        M_AXI_BREADY
);

    // FSM states
    reg [1:0] state;
    localparam ST_AW_W = 2'b00;
    localparam ST_B    = 2'b01;
    localparam ST_DONE = 2'b10;

    always @(posedge aclk) begin
        if (!aresetn) begin
            // Reset all outputs
            M_AXI_AWADDR  <= 32'h00000000;
            M_AXI_AWVALID <= 1'b0;
            M_AXI_WDATA   <= 32'h00000000;
            M_AXI_WSTRB   <= 4'hF;
            M_AXI_WVALID  <= 1'b0;
            M_AXI_BREADY  <= 1'b0;
            state         <= ST_AW_W;
        end else begin
            case (state)

                // Send address + data
                ST_AW_W: begin
                    M_AXI_AWADDR  <= 32'h00000010;   // fixed example address
                    M_AXI_WDATA   <= 32'hDEADBEEF;   // fixed example data
                    M_AXI_AWVALID <= 1'b1;
                    M_AXI_WVALID  <= 1'b1;
                    M_AXI_WSTRB   <= 4'hF;

                    if (M_AXI_AWREADY)
                        M_AXI_AWVALID <= 1'b0;

                    if (M_AXI_WREADY)
                        M_AXI_WVALID <= 1'b0;

                    if (M_AXI_AWREADY && M_AXI_WREADY) begin
                        M_AXI_BREADY <= 1'b1;
                        state        <= ST_B;
                    end
                end

                // Wait for write response
                ST_B: begin
                    if (M_AXI_BVALID) begin
                        M_AXI_BREADY <= 1'b0;
                        state        <= ST_DONE;
                    end
                end

                // Finished
                ST_DONE: begin
                    // No further writes
                end

            endcase
        end
    end

endmodule