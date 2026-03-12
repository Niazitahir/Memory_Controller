module bram_test (
    input  wire clk,
    input  wire reset_n,

    output reg        bram_en,
   (* mark_debug = "true" *) output reg [31:0] bram_addr,
    output reg [31:0] bram_din,
    output reg [3:0]  bram_we,
    input  wire [31:0] bram_dout,
    input  wire        bram_ready,
    input  wire        tester,
    output reg       test_pass 
);

reg tester_latch;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        bram_en   <= 1'b1;
        bram_addr <= 32'h00000000;
        bram_din  <= 32'hDEADBEEF;
        bram_we   <= 4'b1111;
        tester_latch <= 1'b0;
        test_pass <= 1'b0;
    end else begin
        tester_latch <= tester;
        test_pass    <= tester;

        if (tester_latch) begin
            bram_en   <= 1'b1;
            bram_addr <= 32'hC0000000;
            bram_din  <= 32'hDEADBEEF;
            bram_we   <= 4'b1111;
        end
        else 
        begin
            bram_en   <= 1'b0;
            bram_addr <= 32'h00000000;
            bram_din  <= 32'h00000000;
            bram_we   <= 4'b0000;
        end
    end

end

endmodule
