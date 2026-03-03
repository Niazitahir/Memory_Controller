    `timescale 1ns / 1ps

module mem_controller
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
endmodule

    //cause im pedantic and love complicating things
    //note, only a single 32bit register for reads and writes
    //writing more than one thing will overwrite it. 
module axi_lite_slave (
    input         clk,
    input         resetn,

    // AXI-Lite slave interface
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

    // custom signalling
    output            init_write,
    output      [31:0] val_out,
    output      [31:0] n_lsr, 
    output      [31:0] addr_out
);

    localparam BASE_ADDR = 32'h8000_0000;
    localparam MEM_BYTES = 1440;              // 3 cols * 480 bytes
    localparam MEM_WORDS = MEM_BYTES / 4;     // 360 words


    (* ram_style = "block" *)
    reg [31:0] mem [0:MEM_WORDS-1];

    // mode: 01 = byte, 10 = word
    reg [1:0] mode;

    // custom outputs (kept, but not driven here)
    reg [4:0]  n_out;
    reg [31:0] value_in;
    reg [31:0] output_addr;
    reg        mem_fin;

    assign val_out    = value_in;
    assign init_write = mem_fin;
    assign n_lsr      = n_out;
    assign addr_out   = output_addr;


    reg        aw_latched;
    reg        w_latched;
    reg [31:0] awaddr_reg;
    reg [31:0] wdata_reg;
    reg [3:0]  wstrb_reg;
    reg [31:0] aw_offset_reg;


    reg [31:0] offset_reg;
    reg [8:0]  word_index_reg;   // enough for 360 words
    reg [1:0]  byte_sel_reg;

    reg        write_en;         // single write enable pulse

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            S_AXI_AWREADY   <= 1'b0;
            S_AXI_WREADY    <= 1'b0;
            S_AXI_BVALID    <= 1'b0;
            S_AXI_BRESP     <= 2'b00;
            aw_latched      <= 1'b0;
            w_latched       <= 1'b0;
            awaddr_reg      <= 32'd0;
            wdata_reg       <= 32'd0;
            wstrb_reg       <= 4'd0;
            offset_reg      <= 32'd0;
            word_index_reg  <= 9'd0;
            byte_sel_reg    <= 2'd0;
            mode            <= 2'b00;
            mem_fin         <= 1'b0;
            write_en        <= 1'b0;
        end else begin
            mem_fin  <= 1'b0;
            write_en <= 1'b0;

            // ---------------- AW handshake ----------------
            if (!aw_latched && !S_AXI_AWREADY)
                S_AXI_AWREADY <= 1'b1;
            else if (S_AXI_AWREADY && S_AXI_AWVALID) begin
                S_AXI_AWREADY  <= 1'b0;
                aw_latched     <= 1'b1;
                awaddr_reg     <= S_AXI_AWADDR;
                aw_offset_reg <= S_AXI_AWADDR - BASE_ADDR;
                byte_sel_reg  <= (S_AXI_AWADDR - BASE_ADDR) & 32'h3;   // same as [1:0]
                word_index_reg <= (S_AXI_AWADDR - BASE_ADDR) >> 2;

            end else if (aw_latched)
                S_AXI_AWREADY <= 1'b0;

            // ---------------- W handshake ----------------
            if (!w_latched && !S_AXI_WREADY)
                S_AXI_WREADY <= 1'b1;
            else if (S_AXI_WREADY && S_AXI_WVALID) begin
                S_AXI_WREADY <= 1'b0;
                w_latched    <= 1'b1;
                wdata_reg    <= S_AXI_WDATA;
                wstrb_reg    <= S_AXI_WSTRB;
            end else if (w_latched)
                S_AXI_WREADY <= 1'b0;

            // ---------------- Issue write command ----------------
            if (aw_latched && w_latched && !S_AXI_BVALID) begin
                // mode register at BASE_ADDR
                if (awaddr_reg == BASE_ADDR) begin
                    mode <= wdata_reg[1:0];
                end else if (offset_reg < MEM_BYTES) begin
                    write_en <= 1'b1;   // trigger BRAM write port
                end

                mem_fin      <= 1'b1;
                S_AXI_BVALID <= 1'b1;
                S_AXI_BRESP  <= 2'b00; // OKAY
                aw_latched   <= 1'b0;
                w_latched    <= 1'b0;
            end

            // ---------------- B handshake ----------------
            if (S_AXI_BVALID && S_AXI_BREADY)
                S_AXI_BVALID <= 1'b0;
        end
    end


    always @(posedge clk) begin
        if (write_en) begin
            if (word_index_reg < MEM_WORDS) begin
                case (mode)
                    2'b01: begin
                        // BYTE write: use byte_sel_reg to choose lane
                        case (byte_sel_reg)
                            2'b00: if (wstrb_reg[0]) mem[word_index_reg][7:0]   <= wdata_reg[7:0];
                            2'b01: if (wstrb_reg[0]) mem[word_index_reg][15:8]  <= wdata_reg[7:0];
                            2'b10: if (wstrb_reg[0]) mem[word_index_reg][23:16] <= wdata_reg[7:0];
                            2'b11: if (wstrb_reg[0]) mem[word_index_reg][31:24] <= wdata_reg[7:0];
                        endcase
                    end
                    2'b10: begin
                        // WORD write: 4 bytes, controlled by WSTRB
                        if (wstrb_reg[0]) mem[word_index_reg][7:0]   <= wdata_reg[7:0];
                        if (wstrb_reg[1]) mem[word_index_reg][15:8]  <= wdata_reg[15:8];
                        if (wstrb_reg[2]) mem[word_index_reg][23:16] <= wdata_reg[23:16];
                        if (wstrb_reg[3]) mem[word_index_reg][31:24] <= wdata_reg[31:24];
                    end
                    default: ; // no-op
                endcase
            end
        end
    end

    reg        ar_latched;
    reg [31:0] araddr_reg;
    reg [31:0] ar_offset_reg;
    reg [8:0]  ar_word_index_reg;
    reg [1:0]  ar_byte_sel_reg;

    reg [31:0] rd_word;   // BRAM read data (Port B)

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            S_AXI_ARREADY      <= 1'b0;
            S_AXI_RVALID       <= 1'b0;
            S_AXI_RRESP        <= 2'b00;
            S_AXI_RDATA        <= 32'd0;
            ar_latched         <= 1'b0;
            araddr_reg         <= 32'd0;
            ar_offset_reg      <= 32'd0;
            ar_word_index_reg  <= 9'd0;
            ar_byte_sel_reg    <= 2'd0;
        end else begin
            // ---------------- AR handshake ----------------
            if (!ar_latched && !S_AXI_ARREADY)
                S_AXI_ARREADY <= 1'b1;
            else if (S_AXI_ARREADY && S_AXI_ARVALID) begin
                S_AXI_ARREADY      <= 1'b0;
                ar_latched         <= 1'b1;
                araddr_reg         <= S_AXI_ARADDR;
                ar_offset_reg     <= S_AXI_ARADDR - BASE_ADDR;
                ar_byte_sel_reg   <= (S_AXI_ARADDR - BASE_ADDR) & 32'h3;
                ar_word_index_reg <= (S_AXI_ARADDR - BASE_ADDR) >> 2;

            end else if (ar_latched)
                S_AXI_ARREADY <= 1'b0;

            // ---------------- Produce read data ----------------
            if (ar_latched && !S_AXI_RVALID) begin
                if (araddr_reg == BASE_ADDR) begin
                    S_AXI_RDATA <= {30'd0, mode};
                end else if (ar_offset_reg < MEM_BYTES && ar_word_index_reg < MEM_WORDS) begin
                    case (mode)
                        2'b01: begin
                            // BYTE read: select lane from rd_word
                            case (ar_byte_sel_reg)
                                2'b00: S_AXI_RDATA <= {24'd0, rd_word[7:0]};
                                2'b01: S_AXI_RDATA <= {24'd0, rd_word[15:8]};
                                2'b10: S_AXI_RDATA <= {24'd0, rd_word[23:16]};
                                2'b11: S_AXI_RDATA <= {24'd0, rd_word[31:24]};
                            endcase
                        end
                        2'b10: begin
                            // WORD read
                            S_AXI_RDATA <= rd_word;
                        end
                        default: S_AXI_RDATA <= 32'd0;
                    endcase
                end else begin
                    S_AXI_RDATA <= 32'd0;
                end

                S_AXI_RVALID <= 1'b1;
                S_AXI_RRESP  <= 2'b00; // OKAY
                ar_latched   <= 1'b0;
            end

            // ---------------- R handshake ----------------
            if (S_AXI_RVALID && S_AXI_RREADY)
                S_AXI_RVALID <= 1'b0;
        end
    end


    always @(posedge clk) begin
        if (ar_latched && ar_offset_reg < MEM_BYTES && ar_word_index_reg < MEM_WORDS)
            rd_word <= mem[ar_word_index_reg];
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
    output reg        M_AXI_BREADY,
    //custom signalling
    input      [31:0] value_out,
    input         init_write,
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

