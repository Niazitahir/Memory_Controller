    `timescale 1ns / 1ps

module mem_controller
(
    input aclk,
    input aresetn,

    // -------------------------
    // Port A user interface
    // -------------------------
    input  wire        a_en,
    input  wire        a_we,           // 1 = write, 0 = read
    input  wire [3:0]  a_addr,
    input  wire [31:0] a_din,
    output reg  [31:0] a_dout,

    // -------------------------
    // Port B user interface
    // -------------------------
    input  wire        b_en,
    input  wire        b_we,           // 1 = write, 0 = read
    input  wire [3:0]  b_addr,
    input  wire [31:0] b_din,
    output reg  [31:0] b_dout,

    // -------------------------
    // Connections to BRAM IP
    // -------------------------
    output wire        bram_ena,
    output wire [3:0]  bram_wea,
    output wire [3:0]  bram_addra,
    output wire [31:0] bram_dina,
    input  wire [31:0] bram_douta,
    output wire        bram_rsta,

    output wire        bram_enb,
    output wire [3:0]  bram_web,
    output wire [3:0]  bram_addrb,
    output wire [31:0] bram_dinb,
    input  wire [31:0] bram_doutb,
    output wire        bram_rstb,
    
    //Victor inputs
    input bram_en,
    input [31:0] bram_addr,
    input [31:0] bram_din,
    input [3:0] bram_we,
    output [31:0] bram_dout,



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
    output        M_AXI_BREADY,
    output [31:0] a_read_val_sig,
    output [31:0] b_read_val_sig
);




    wire [31:0] val_out;
    wire init_out;
    wire [31:0] lsr_in;
    wire [31:0] lsr_n;
    wire [31:0] addr;

  // ------------------------------------------------------------
    wire        a_en_sig;
    wire        a_we_sig;
    wire [3:0]  a_addr_sig;
    wire [31:0] a_din_sig;
    wire [31:0] a_dout_sig;

    wire        b_en_sig;
    wire        b_we_sig;
    wire [3:0]  b_addr_sig;
    wire [31:0] b_din_sig;
    wire [31:0] b_dout_sig;

    // Optional: expose readback values for debugging



    bram_driver_stimulus stimmy(
        .clk(aclk),
        .reset_n(aresetn),
        // Outputs for observation (ILA, LEDs, etc.)
        .a_read_value(a_read_val_sig),
        .b_read_value(b_read_val_sig),

        .a_en(a_en_sig),
        .a_we(a_we_sig),
        .a_addr(a_addr_sig),
        .a_din(a_din_sig),
        .a_dout(a_dout_sig),

        .b_en(b_en_sig),
        .b_we(b_we_sig),
        .b_addr(b_addr_sig),
        .b_din(b_din_sig),
        .b_dout(b_dout_sig)

    );
    bram_dualport_driver driver_inst (
        .clk(aclk),
        .reset_n(aresetn),

        // User-side (from stimulus)
        .a_en(a_en_sig),
        .a_we(a_we_sig),
        .a_addr(a_addr_sig),
        .a_din(a_din_sig),
        .a_dout(a_dout_sig),

        .b_en(b_en_sig),
        .b_we(b_we_sig),
        .b_addr(b_addr_sig),
        .b_din(b_din_sig),
        .b_dout(b_dout_sig),



        // BRAM Port A pins
        .bram_ena(bram_ena),
        .bram_wea(bram_wea),
        .bram_addra(bram_addra),
        .bram_dina(bram_dina),
        .bram_douta(bram_douta),
        .bram_rsta(bram_rsta),

        // BRAM Port B pins
        .bram_enb(bram_enb),
        .bram_web(bram_web),
        .bram_addrb(bram_addrb),
        .bram_dinb(bram_dinb),
        .bram_doutb(bram_doutb),
        .bram_rstb(bram_rstb)
    );

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
        //bram interface to victor
        .bram_en(bram_en),
        .bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_we(bram_we),
        .bram_dout(bram_dout),

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




module bram_dualport_driver (
    input  wire        clk,
    input  wire        reset_n,


    // Port A user interface

    input  wire        a_en,
    input  wire        a_we,           // 1 = write, 0 = read
    input  wire [3:0]  a_addr,
    input  wire [31:0] a_din,
    output reg  [31:0] a_dout,


    // Port B user interface

    input  wire        b_en,
    input  wire        b_we,           // 1 = write, 0 = read
    input  wire [3:0]  b_addr,
    input  wire [31:0] b_din,
    output reg  [31:0] b_dout,


    // Connections to BRAM IP

    output wire        bram_ena,
    output wire [3:0]  bram_wea,
    output wire [3:0]  bram_addra,
    output wire [31:0] bram_dina,
    input  wire [31:0] bram_douta,
    output wire        bram_rsta,

    output wire        bram_enb,
    output wire [3:0]  bram_web,
    output wire [3:0]  bram_addrb,
    output wire [31:0] bram_dinb,
    input  wire [31:0] bram_doutb,
    output wire        bram_rstb
);


    // Port A wiring

    assign bram_ena   = a_en;
    assign bram_wea   = a_we ? 4'b1111 : 4'b0000;
    assign bram_addra = a_addr;
    assign bram_dina  = a_din;
    assign bram_rsta  = reset_n;

    // BRAM is synchronous → register output
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            a_dout <= 32'b0;
        else if (a_en && !a_we)
            a_dout <= bram_douta;
    end


    // Port B wiring

    assign bram_enb   = b_en;
    assign bram_web   = b_we ? 4'b1111 : 4'b0000;
    assign bram_addrb = b_addr;
    assign bram_dinb  = b_din;
    assign bram_rstb  = reset_n;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            b_dout <= 32'b0;
        else if (b_en && !b_we)
            b_dout <= bram_doutb;
    end

endmodule

module bram_driver_stimulus (
    input  wire        clk,
    input  wire        reset_n,

    // Outputs for observation (ILA, LEDs, etc.)
    output reg [31:0]  a_read_value,
    output reg [31:0]  b_read_value,

    // User-side interface to bram_dualport_driver
    output reg         a_en,
    output reg         a_we,
    output reg [3:0]   a_addr,
    output reg [31:0]  a_din,
    input  wire [31:0] a_dout,

    output reg         b_en,
    output reg         b_we,
    output reg [3:0]   b_addr,
    output reg [31:0]  b_din,
    input  wire [31:0] b_dout
);

    // Simple FSM states
    localparam IDLE     = 2'd0;
    localparam WRITE    = 2'd1;
    localparam READ     = 2'd2;
    localparam HOLD     = 2'd3;

    reg [1:0] state;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Reset everything
            state        <= IDLE;
            a_en         <= 0;
            a_we         <= 0;
            a_addr       <= 0;
            a_din        <= 0;
            b_en         <= 0;
            b_we         <= 0;
            b_addr       <= 0;
            b_din        <= 0;
            a_read_value <= 0;
            b_read_value <= 0;

        end else begin
            case (state)
                IDLE: begin
                    a_en   <= 1;
                    b_en   <= 1;
                    a_we   <= 1;   // write mode
                    b_we   <= 1;
                    a_addr <= 4'h0;
                    b_addr <= 4'h1;
                    a_din  <= 32'hDEADBEEF;
                    b_din  <= 32'hDEADBEEF;
                    state  <= WRITE;
                end
                WRITE: begin
                    // One cycle of write is enough
                    a_we <= 0;   // switch to read mode
                    b_we <= 0;
                    state <= READ;
                end
                READ: begin
                    a_addr <= 4'h0;
                    b_addr <= 4'h1;
                    // BRAM is synchronous → capture next cycle
                    a_read_value <= a_dout;
                    b_read_value <= b_dout;
                    state <= HOLD;
                end
                HOLD: begin
                    // Nothing changes; values remain visible
                end

            endcase
        end
    end

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
    localparam MEM_BYTES = 1;              // 3 cols * 480 bytes = 1440 bytes total, but we'll just use the first word for simplicity
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

    input bram_en,
    input [31:0] bram_addr,
    input [31:0] bram_din,
    input [3:0] bram_we,
    output [31:0] bram_dout,
    output reg bram_ready,


    input         aclk,
    input         aresetn,

    output reg [31:0] M_AXI_AWADDR,
    output reg        M_AXI_AWVALID,
    input             M_AXI_AWREADY,

    output reg [31:0] M_AXI_WDATA,
    output reg [3:0]  M_AXI_WSTRB,
    output reg        M_AXI_WVALID,
    input             M_AXI_WREADY,

   (* mark_debug = "true" *) input      [1:0]  M_AXI_BRESP,
    input             M_AXI_BVALID,
    output reg        M_AXI_BREADY,
    //custom signalling
    input      [31:0] value_out,
    input         init_write,
    input       [31:0] addr_in,
    output reg [1:0] state_led
);

    reg [1:0] state;
    reg [31:0] val_save;
    reg [31:0] latch_addr;
    reg  [1:0] bresp_latched;
    reg [3:0] bram_we_latched;
    //FSM Three stages
    localparam ST_AW_W = 2'b00;
    localparam ST_B    = 2'b01;
    localparam ST_DONE = 2'b10;

    localparam ST_AWBRAM = 2'b11; // not used, but could be for direct BRAM access without AXI handshakes


    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            M_AXI_AWADDR  <= 32'h00000000;
            M_AXI_AWVALID <= 1'b0;
            M_AXI_WDATA   <= 32'h00000000;
            M_AXI_WSTRB   <= 4'hF;
            M_AXI_WVALID  <= 1'b0;
            M_AXI_BREADY  <= 1'b1;
            latch_addr <= 32'h00000000;
            bresp_latched <= 2'b00;
            state         <= ST_DONE;
            state_led <= 2'b00;

        end else begin
            state_led <= state; // for debugging: show current state on LEDs
            bresp_latched <= M_AXI_BRESP; // latch BRESP for potential error handling (not implemented here)
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
                        M_AXI_BREADY <= 1'b1;
                        state        <= ST_DONE;
                    end
                    
                end
                ST_AWBRAM: begin
                    bram_ready <= 1'b0;
                    latch_addr <= bram_addr;
                    val_save = bram_din;    
                    bram_we_latched <= bram_we; // capture BRAM write enable for later use
                    // drive address + data and hold VALID until handshake
                    M_AXI_AWADDR  <= latch_addr;   // matches mem[0]
                    M_AXI_WDATA   <= val_save;
                    M_AXI_WSTRB   <= bram_we_latched;

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
                ST_DONE: begin
               
                bram_ready <= 1'b1;
                    //possible issue: Congestion if bram_en on constantly, stifling init_write
                if(bram_en) begin
                    state <= ST_AWBRAM;
                    M_AXI_BREADY <= 1'b1; // always ready to accept response, but not driving new transactions
                end
                else if (init_write)
                    state <= ST_AW_W;

                end
                

            endcase
        end
    end

endmodule

