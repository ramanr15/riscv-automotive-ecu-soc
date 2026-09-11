`timescale 1ns / 1ps
// ============================================================
// uart_ctrl : 8-N-1 UART with TX and RX FIFOs
// Person 2 - Step 4
//
// Uses a 16x oversample tick so the receiver samples the
// middle of each bit, exactly like the CAN bit timing idea but
// simpler: no resynchronisation beyond the start bit.
//
//   OSDIV = (f_clk / (16 * baud)) - 1
//   e.g. 100 MHz, 115200 baud -> 100e6/(16*115200) = 54.25 -> 54
//
// REGISTER MAP
//   0x00  DATA    RW  write = push TX FIFO, read = pop RX FIFO
//   0x04  STATUS  RO  [0] tx_full   [1] tx_empty
//                     [2] rx_full   [3] rx_empty
//                     [4] framing_error (sticky)
//                     [5] overrun_error (sticky)
//                     [6] tx_busy
//   0x08  OSDIV   RW  [15:0] oversample divider - 1
//   0x0C  CTRL    RW  [0] rx interrupt enable
//                     [1] tx interrupt enable (fires when TX empty)
//                     [2] clear sticky errors (self-clearing)
// ============================================================
module uart_ctrl #(
    parameter FIFO_LOG2 = 4          // 16-byte FIFOs
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        sel,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output reg  [31:0] prdata,

    input  wire        uart_rx,
    output wire        uart_tx,

    output wire        irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    // ------------------------------------------------------------
    // Configuration registers
    // ------------------------------------------------------------
    reg [15:0] osdiv;
    reg        rx_irq_en;
    reg        tx_irq_en;
    reg        clear_err;

    // ------------------------------------------------------------
    // 16x oversample tick generator
    // ------------------------------------------------------------
    reg [15:0] os_cnt;
    wire       os_tick;

    assign os_tick = (os_cnt == 16'd0);

    always @(posedge clk) begin
        if (reset)            os_cnt <= 16'd0;
        else if (os_cnt == 0) os_cnt <= osdiv;
        else                  os_cnt <= os_cnt - 16'd1;
    end

    // ------------------------------------------------------------
    // FIFOs (Step 3 template)
    // ------------------------------------------------------------
    wire        tx_full, tx_empty;
    wire [7:0]  tx_dout;
    reg         tx_pop;
    wire        tx_push;

    wire        rx_full, rx_empty;
    wire [7:0]  rx_dout;
    wire        rx_pop;
    reg         rx_push;
    reg  [7:0]  rx_byte;

    assign tx_push = wr && (reg_index == 6'h00);
    assign rx_pop  = rd && (reg_index == 6'h00);

    fifo_sync #(.WIDTH(8), .DEPTH_LOG2(FIFO_LOG2)) TX_FIFO (
        .clk(clk), .reset(reset),
        .push(tx_push), .din(wdata[7:0]),
        .pop(tx_pop),   .dout(tx_dout),
        .full(tx_full), .empty(tx_empty), .count()
    );

    fifo_sync #(.WIDTH(8), .DEPTH_LOG2(FIFO_LOG2)) RX_FIFO (
        .clk(clk), .reset(reset),
        .push(rx_push), .din(rx_byte),
        .pop(rx_pop),   .dout(rx_dout),
        .full(rx_full), .empty(rx_empty), .count()
    );

    // ------------------------------------------------------------
    // TRANSMITTER
    // start bit + 8 data bits (LSB first) + stop bit
    // ------------------------------------------------------------
    localparam TX_IDLE  = 2'd0;
    localparam TX_START = 2'd1;
    localparam TX_DATA  = 2'd2;
    localparam TX_STOP  = 2'd3;

    reg [1:0] tx_state;
    reg [3:0] tx_os;
    reg [2:0] tx_bit;
    reg [7:0] tx_sr;
    reg       tx_line;

    assign uart_tx = tx_line;

    always @(posedge clk) begin
        if (reset) begin
            tx_state <= TX_IDLE;
            tx_line  <= 1'b1;
            tx_os    <= 4'd0;
            tx_bit   <= 3'd0;
            tx_sr    <= 8'd0;
            tx_pop   <= 1'b0;
        end
        else begin
            tx_pop <= 1'b0;

            if (os_tick) begin
                case (tx_state)

                    TX_IDLE: begin
                        tx_line <= 1'b1;
                        if (!tx_empty) begin
                            tx_sr    <= tx_dout;
                            tx_pop   <= 1'b1;
                            tx_line  <= 1'b0;      // start bit
                            tx_os    <= 4'd0;
                            tx_state <= TX_START;
                        end
                    end

                    TX_START: begin
                        tx_os <= tx_os + 4'd1;
                        if (tx_os == 4'd15) begin
                            tx_os    <= 4'd0;
                            tx_bit   <= 3'd0;
                            tx_line  <= tx_sr[0];
                            tx_state <= TX_DATA;
                        end
                    end

                    TX_DATA: begin
                        tx_os <= tx_os + 4'd1;
                        if (tx_os == 4'd15) begin
                            tx_os <= 4'd0;
                            if (tx_bit == 3'd7) begin
                                tx_line  <= 1'b1;  // stop bit
                                tx_state <= TX_STOP;
                            end
                            else begin
                                tx_bit  <= tx_bit + 3'd1;
                                tx_sr   <= {1'b0, tx_sr[7:1]};
                                tx_line <= tx_sr[1];
                            end
                        end
                    end

                    TX_STOP: begin
                        tx_os <= tx_os + 4'd1;
                        if (tx_os == 4'd15) begin
                            tx_os    <= 4'd0;
                            tx_state <= TX_IDLE;
                        end
                    end

                endcase
            end
        end
    end

    wire tx_busy;
    assign tx_busy = (tx_state != TX_IDLE) | ~tx_empty;

    // ------------------------------------------------------------
    // RECEIVER
    // Two-flop synchroniser, then start-bit edge detect and
    // mid-bit sampling on the oversample tick.
    // ------------------------------------------------------------
    reg rx_s0, rx_s1;

    always @(posedge clk) begin
        if (reset) begin
            rx_s0 <= 1'b1;
            rx_s1 <= 1'b1;
        end
        else begin
            rx_s0 <= uart_rx;
            rx_s1 <= rx_s0;
        end
    end

    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0] rx_state;
    reg [3:0] rx_os;
    reg [2:0] rx_bit;
    reg [7:0] rx_sr;

    reg framing_error;
    reg overrun_error;

    always @(posedge clk) begin
        if (reset) begin
            rx_state      <= RX_IDLE;
            rx_os         <= 4'd0;
            rx_bit        <= 3'd0;
            rx_sr         <= 8'd0;
            rx_push       <= 1'b0;
            rx_byte       <= 8'd0;
            framing_error <= 1'b0;
            overrun_error <= 1'b0;
        end
        else begin
            rx_push <= 1'b0;

            if (clear_err) begin
                framing_error <= 1'b0;
                overrun_error <= 1'b0;
            end

            if (os_tick) begin
                case (rx_state)

                    RX_IDLE: begin
                        if (rx_s1 == 1'b0) begin   // falling edge / start
                            rx_os    <= 4'd0;
                            rx_state <= RX_START;
                        end
                    end

                    RX_START: begin
                        rx_os <= rx_os + 4'd1;
                        if (rx_os == 4'd7) begin   // middle of start bit
                            if (rx_s1 == 1'b0) begin
                                rx_os    <= 4'd0;
                                rx_bit   <= 3'd0;
                                rx_state <= RX_DATA;
                            end
                            else begin
                                rx_state <= RX_IDLE;  // glitch, abort
                            end
                        end
                    end

                    RX_DATA: begin
                        rx_os <= rx_os + 4'd1;
                        if (rx_os == 4'd15) begin  // middle of data bit
                            rx_os <= 4'd0;
                            rx_sr <= {rx_s1, rx_sr[7:1]};
                            if (rx_bit == 3'd7)
                                rx_state <= RX_STOP;
                            else
                                rx_bit <= rx_bit + 3'd1;
                        end
                    end

                    RX_STOP: begin
                        rx_os <= rx_os + 4'd1;
                        if (rx_os == 4'd15) begin
                            rx_os    <= 4'd0;
                            rx_state <= RX_IDLE;

                            if (rx_s1 == 1'b1) begin
                                if (rx_full)
                                    overrun_error <= 1'b1;
                                else begin
                                    rx_byte <= rx_sr;
                                    rx_push <= 1'b1;
                                end
                            end
                            else begin
                                framing_error <= 1'b1;
                            end
                        end
                    end

                endcase
            end
        end
    end

    // ------------------------------------------------------------
    // Control register
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            osdiv     <= 16'd53;   // 100 MHz / (16 * 115200)
            rx_irq_en <= 1'b0;
            tx_irq_en <= 1'b0;
            clear_err <= 1'b0;
        end
        else begin
            clear_err <= 1'b0;
            if (wr) begin
                case (reg_index)
                    6'h02: osdiv <= wdata[15:0];
                    6'h03: begin
                        rx_irq_en <= wdata[0];
                        tx_irq_en <= wdata[1];
                        clear_err <= wdata[2];
                    end
                    default: ;
                endcase
            end
        end
    end

    assign irq = (rx_irq_en & ~rx_empty) |
                 (tx_irq_en &  tx_empty & (tx_state == TX_IDLE));

    // ------------------------------------------------------------
    // Read port
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00: prdata <= {24'b0, rx_dout};
                6'h01: prdata <= {25'b0, tx_busy,
                                  overrun_error, framing_error,
                                  rx_empty, rx_full,
                                  tx_empty, tx_full};
                6'h02: prdata <= {16'b0, osdiv};
                6'h03: prdata <= {30'b0, tx_irq_en, rx_irq_en};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
