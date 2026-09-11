`timescale 1ns / 1ps
// ============================================================
// i2c_ctrl : I2C master
// Person 2 - Step 6
//
// Command driven byte-level engine. Software issues one
// command at a time and waits for DONE (poll or interrupt).
// The engine covers: Idle, Start, Address/Data write, ACK
// check, Data read, ACK generate, Stop.
//
// OPEN DRAIN CONVENTION
//   scl_low / sda_low = 1 means actively pull the line to 0.
//   The top level resolves this as:
//       assign scl = scl_low ? 1'b0 : 1'bz;
//       assign sda = sda_low ? 1'b0 : 1'bz;
//   scl_i / sda_i are the synchronised readbacks of the pads.
//
// Clock stretching is handled by never leaving a phase in
// which SCL has been released until scl_i actually reads high.
//
//   SCL frequency = f_clk / (4 * (CLKDIV + 1))
//   100 MHz, 100 kHz -> CLKDIV = 249
//   100 MHz, 400 kHz -> CLKDIV = 61
//
// REGISTER MAP
//   0x00  CTRL    RW  [15:0] CLKDIV, [16] interrupt enable
//   0x04  CMD     WO  [0] START   (generate start / repeated start)
//                     [1] WRITE   (send TXDATA, capture ACK)
//                     [2] READ    (receive into RXDATA)
//                     [3] STOP    (generate stop)
//                     [4] ACK value to send after a READ
//                         (0 = ACK, 1 = NACK / last byte)
//                     Commands execute in the order
//                     START -> WRITE/READ -> STOP within one
//                     command word, so a full "start, send
//                     address" is a single write.
//   0x08  TXDATA  RW  byte to transmit (address byte = {addr7, rw})
//   0x0C  RXDATA  RO  last received byte
//   0x10  STATUS  RW  [0] busy
//                     [1] ack_error  (NACK where ACK expected)
//                     [2] done       (write 1 to clear)
// ============================================================
module i2c_ctrl (
    input  wire        clk,
    input  wire        reset,

    input  wire        sel,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output reg  [31:0] prdata,

    input  wire        scl_i,
    input  wire        sda_i,
    output reg         scl_low,
    output reg         sda_low,

    output wire        irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    // ------------------------------------------------------------
    // Input synchronisers
    // ------------------------------------------------------------
    reg scl_s0, scl_s1, sda_s0, sda_s1;
    always @(posedge clk) begin
        if (reset) begin
            scl_s0 <= 1'b1; scl_s1 <= 1'b1;
            sda_s0 <= 1'b1; sda_s1 <= 1'b1;
        end
        else begin
            scl_s0 <= scl_i; scl_s1 <= scl_s0;
            sda_s0 <= sda_i; sda_s1 <= sda_s0;
        end
    end

    // ------------------------------------------------------------
    // Configuration and command latches
    // ------------------------------------------------------------
    reg [15:0] clkdiv;
    reg        irq_enable;

    reg [7:0]  tx_data;
    reg [7:0]  rx_data;

    reg        cmd_start;
    reg        cmd_write;
    reg        cmd_read;
    reg        cmd_stop;
    reg        cmd_ack;

    reg        busy;
    reg        done;
    reg        ack_error;

    // ------------------------------------------------------------
    // Quarter-bit-period tick
    // ------------------------------------------------------------
    reg  [15:0] div_cnt;
    wire        q_tick;

    assign q_tick = (div_cnt == 16'd0);

    always @(posedge clk) begin
        if (reset || !busy)     div_cnt <= 16'd0;
        else if (div_cnt == 0)  div_cnt <= clkdiv;
        else                    div_cnt <= div_cnt - 16'd1;
    end

    // ------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------
    localparam ST_IDLE    = 4'd0;
    localparam ST_START   = 4'd1;
    localparam ST_WR_BIT  = 4'd2;
    localparam ST_WR_ACK  = 4'd3;
    localparam ST_RD_BIT  = 4'd4;
    localparam ST_RD_ACK  = 4'd5;
    localparam ST_STOP    = 4'd6;
    localparam ST_FINISH  = 4'd7;

    reg [3:0] state;
    reg [1:0] phase;
    reg [2:0] bit_cnt;
    reg [7:0] shift;

    // A phase may only advance once SCL has genuinely gone high,
    // which is where a stretching slave holds us up.
    wire stretch_ok;
    assign stretch_ok = scl_s1;

    always @(posedge clk) begin
        if (reset) begin
            state     <= ST_IDLE;
            phase     <= 2'd0;
            bit_cnt   <= 3'd0;
            shift     <= 8'd0;
            scl_low   <= 1'b0;
            sda_low   <= 1'b0;
            busy      <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
            rx_data   <= 8'd0;
            cmd_start <= 1'b0;
            cmd_write <= 1'b0;
            cmd_read  <= 1'b0;
            cmd_stop  <= 1'b0;
            cmd_ack   <= 1'b0;
        end
        else begin

            // ---------------- command issue ----------------
            if (wr && (reg_index == 6'h01) && !busy) begin
                cmd_start <= wdata[0];
                cmd_write <= wdata[1];
                cmd_read  <= wdata[2];
                cmd_stop  <= wdata[3];
                cmd_ack   <= wdata[4];
                shift     <= tx_data;
                bit_cnt   <= 3'd0;
                phase     <= 2'd0;
                busy      <= 1'b1;
                ack_error <= 1'b0;

                if (wdata[0])      state <= ST_START;
                else if (wdata[1]) state <= ST_WR_BIT;
                else if (wdata[2]) state <= ST_RD_BIT;
                else if (wdata[3]) state <= ST_STOP;
                else begin
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end
            end

            // ---------------- software clears DONE ----------------
            if (wr && (reg_index == 6'h04) && wdata[2])
                done <= 1'b0;

            // ---------------- bit engine ----------------
            if (busy && q_tick) begin
                case (state)

                    // ------------------------------------------
                    // START / repeated START
                    //   release SDA, release SCL, pull SDA low
                    //   while SCL is high, then pull SCL low
                    // ------------------------------------------
                    ST_START: begin
                        case (phase)
                            2'd0: begin
                                sda_low <= 1'b0;
                                scl_low <= 1'b1;
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                sda_low <= 1'b1;      // START condition
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_low <= 1'b1;
                                phase   <= 2'd0;
                                shift   <= tx_data;
                                bit_cnt <= 3'd0;
                                if (cmd_write)     state <= ST_WR_BIT;
                                else if (cmd_read) state <= ST_RD_BIT;
                                else if (cmd_stop) state <= ST_STOP;
                                else               state <= ST_FINISH;
                            end
                        endcase
                    end

                    // ------------------------------------------
                    // WRITE one bit, MSB first
                    // ------------------------------------------
                    ST_WR_BIT: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= ~shift[7];   // drive 0 for a 0
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                scl_low <= 1'b1;
                                phase   <= 2'd0;
                                shift   <= {shift[6:0], 1'b0};
                                if (bit_cnt == 3'd7) begin
                                    bit_cnt <= 3'd0;
                                    state   <= ST_WR_ACK;
                                end
                                else begin
                                    bit_cnt <= bit_cnt + 3'd1;
                                end
                            end
                        endcase
                    end

                    // ------------------------------------------
                    // Read the slave's ACK bit
                    // ------------------------------------------
                    ST_WR_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= 1'b0;        // release SDA
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                if (sda_s1) ack_error <= 1'b1;  // NACK
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                scl_low <= 1'b1;
                                phase   <= 2'd0;
                                if (cmd_read)      state <= ST_RD_BIT;
                                else if (cmd_stop) state <= ST_STOP;
                                else               state <= ST_FINISH;
                            end
                        endcase
                    end

                    // ------------------------------------------
                    // READ one bit
                    // ------------------------------------------
                    ST_RD_BIT: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= 1'b0;        // release for slave
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                rx_data <= {rx_data[6:0], sda_s1};
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_low <= 1'b1;
                                phase   <= 2'd0;
                                if (bit_cnt == 3'd7) begin
                                    bit_cnt <= 3'd0;
                                    state   <= ST_RD_ACK;
                                end
                                else begin
                                    bit_cnt <= bit_cnt + 3'd1;
                                end
                            end
                        endcase
                    end

                    // ------------------------------------------
                    // Send our ACK / NACK back to the slave
                    // ------------------------------------------
                    ST_RD_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= ~cmd_ack;    // ack=0 -> pull low
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                scl_low <= 1'b1;
                                phase   <= 2'd0;
                                if (cmd_stop) state <= ST_STOP;
                                else          state <= ST_FINISH;
                            end
                        endcase
                    end

                    // ------------------------------------------
                    // STOP : release SDA low-to-high while SCL high
                    // ------------------------------------------
                    ST_STOP: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= 1'b1;
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                                if (stretch_ok) phase <= 2'd2;
                            end
                            2'd2: begin
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                sda_low <= 1'b0;       // STOP condition
                                phase   <= 2'd0;
                                state   <= ST_FINISH;
                            end
                        endcase
                    end

                    ST_FINISH: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: begin
                        state <= ST_IDLE;
                        busy  <= 1'b0;
                    end

                endcase
            end
        end
    end

    // ------------------------------------------------------------
    // Configuration and TX data registers
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            clkdiv     <= 16'd249;     // 100 kHz from 100 MHz
            irq_enable <= 1'b0;
            tx_data    <= 8'd0;
        end
        else if (wr) begin
            case (reg_index)
                6'h00: begin
                    clkdiv     <= wdata[15:0];
                    irq_enable <= wdata[16];
                end
                6'h02: tx_data <= wdata[7:0];
                default: ;
            endcase
        end
    end

    assign irq = done & irq_enable;

    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {15'b0, irq_enable, clkdiv};
                6'h02:   prdata <= {24'b0, tx_data};
                6'h03:   prdata <= {24'b0, rx_data};
                6'h04:   prdata <= {29'b0, done, ack_error, busy};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
