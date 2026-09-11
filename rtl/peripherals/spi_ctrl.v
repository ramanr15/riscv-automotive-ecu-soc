`timescale 1ns / 1ps
// ============================================================
// spi_ctrl : SPI master, 8-bit, configurable CPOL / CPHA
// Person 2 - Step 5
//
// Master mode only. Writing DATA starts a transfer: 8 bits go
// out on MOSI while 8 bits come in from MISO. Chip select is
// driven low for the duration of the transfer when in auto
// mode, or held under software control for multi-byte frames.
//
//   SCLK frequency = f_clk / (2 * (CLKDIV + 1))
//
// REGISTER MAP
//   0x00  DATA    RW  write = start transfer, read = received byte
//   0x04  CTRL    RW  [7:0]  clock divider
//                     [8]    CPOL
//                     [9]    CPHA
//                     [10]   interrupt enable
//                     [11]   auto chip select
//                     [15:12] manual CS lines (active low, 4 slaves)
//   0x08  STATUS  RW  [0] busy, [1] done (write 1 to clear)
// ============================================================
module spi_ctrl #(
    parameter NUM_CS = 4
)(
    input  wire              clk,
    input  wire              reset,

    input  wire              sel,
    input  wire [31:0]       addr,
    input  wire [31:0]       wdata,
    input  wire              we,
    input  wire              re,
    output reg  [31:0]       prdata,

    output wire              spi_sclk,
    output wire              spi_mosi,
    input  wire              spi_miso,
    output reg  [NUM_CS-1:0] spi_cs_n,

    output wire              irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg [7:0] clkdiv;
    reg       cpol;
    reg       cpha;
    reg       irq_enable;
    reg       cs_auto;
    reg [NUM_CS-1:0] cs_manual;

    reg       busy;
    reg       done;

    reg [7:0] shift_tx;
    reg [7:0] shift_rx;
    reg [3:0] edge_idx;
    reg       sclk_int;

    reg [7:0] div_cnt;
    wire      edge_tick;

    assign edge_tick = busy && (div_cnt == 8'd0);

    assign spi_sclk = sclk_int;
    assign spi_mosi = shift_tx[7];

    // ------------------------------------------------------------
    // MISO synchroniser (external asynchronous input)
    // ------------------------------------------------------------
    reg miso_s0, miso_s1;
    always @(posedge clk) begin
        if (reset) begin miso_s0 <= 1'b0; miso_s1 <= 1'b0; end
        else       begin miso_s0 <= spi_miso; miso_s1 <= miso_s0; end
    end

    // ------------------------------------------------------------
    // Clock divider
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset || !busy)      div_cnt <= 8'd0;
        else if (div_cnt == 0)   div_cnt <= clkdiv;
        else                     div_cnt <= div_cnt - 8'd1;
    end

    // ------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            clkdiv     <= 8'd7;
            cpol       <= 1'b0;
            cpha       <= 1'b0;
            irq_enable <= 1'b0;
            cs_auto    <= 1'b1;
            cs_manual  <= {NUM_CS{1'b1}};
        end
        else if (wr && (reg_index == 6'h01)) begin
            clkdiv     <= wdata[7:0];
            cpol       <= wdata[8];
            cpha       <= wdata[9];
            irq_enable <= wdata[10];
            cs_auto    <= wdata[11];
            cs_manual  <= wdata[15:12];
        end
    end

    // ------------------------------------------------------------
    // Shift engine
    //   CPHA = 0 : sample on the leading edge, change on trailing
    //   CPHA = 1 : change on the leading edge, sample on trailing
    // 16 edges = 8 bits.
    // ------------------------------------------------------------
    wire start_transfer;
    assign start_transfer = wr && (reg_index == 6'h00) && !busy;

    always @(posedge clk) begin
        if (reset) begin
            busy     <= 1'b0;
            done     <= 1'b0;
            sclk_int <= 1'b0;
            edge_idx <= 4'd0;
            shift_tx <= 8'd0;
            shift_rx <= 8'd0;
            spi_cs_n <= {NUM_CS{1'b1}};
        end
        else begin

            // Software clears the done flag (hardware set below wins).
            if (wr && (reg_index == 6'h02) && wdata[1])
                done <= 1'b0;

            if (!busy) begin
                sclk_int <= cpol;
                spi_cs_n <= cs_auto ? {NUM_CS{1'b1}} : cs_manual;
            end

            if (start_transfer) begin
                shift_tx <= wdata[7:0];
                shift_rx <= 8'd0;
                edge_idx <= 4'd0;
                busy     <= 1'b1;
                sclk_int <= cpol;
                spi_cs_n <= cs_auto ? {{(NUM_CS-1){1'b1}}, 1'b0}
                                    : cs_manual;
            end
            else if (edge_tick) begin

                if (cpha == 1'b0) begin
                    if (!edge_idx[0])
                        shift_rx <= {shift_rx[6:0], miso_s1};   // leading
                    else
                        shift_tx <= {shift_tx[6:0], 1'b0};      // trailing
                end
                else begin
                    if (!edge_idx[0]) begin
                        if (edge_idx != 4'd0)
                            shift_tx <= {shift_tx[6:0], 1'b0};  // leading
                    end
                    else begin
                        shift_rx <= {shift_rx[6:0], miso_s1};   // trailing
                    end
                end

                sclk_int <= ~sclk_int;
                edge_idx <= edge_idx + 4'd1;

                if (edge_idx == 4'd15) begin
                    busy     <= 1'b0;
                    done     <= 1'b1;
                    sclk_int <= cpol;
                    if (cs_auto) spi_cs_n <= {NUM_CS{1'b1}};
                end
            end

        end
    end

    assign irq = done & irq_enable;

    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {24'b0, shift_rx};
                6'h01:   prdata <= {16'b0, cs_manual, cs_auto,
                                    irq_enable, cpha, cpol, clkdiv};
                6'h02:   prdata <= {30'b0, done, busy};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
