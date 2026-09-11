`timescale 1ns / 1ps
// ============================================================
// pwm_ctrl : Multi-channel PWM built on timer_core
// Person 2 - Step 9
//
// Reuses the same free-running counter core as the Timer. All
// channels share one counter and one period, and each has its
// own duty register, producing leading-edge-aligned PWM:
//   output high while (count < duty), low otherwise.
//
// REGISTER MAP
//   0x00  CTRL      RW  [0] enable, [1] output invert
//   0x04  PRESCALE  RW  [15:0] clock divider - 1
//   0x08  PERIOD    RW  counter wrap value
//   0x0C  COUNT     RO  live counter value
//   0x10  DUTY0     RW  channel 0 duty
//   0x14  DUTY1     RW  channel 1 duty
//   0x18  DUTY2     RW  channel 2 duty
//   0x1C  DUTY3     RW  channel 3 duty
//   0x20  CHEN      RW  [3:0] per-channel output enable
//
// A duty of 0 gives a permanently low output; a duty greater
// than PERIOD gives a permanently high output.
// ============================================================
module pwm_ctrl #(
    parameter CHANNELS = 4
)(
    input  wire                 clk,
    input  wire                 reset,

    input  wire                 sel,
    input  wire [31:0]          addr,
    input  wire [31:0]          wdata,
    input  wire                 we,
    input  wire                 re,
    output reg  [31:0]          prdata,

    output reg  [CHANNELS-1:0]  pwm_out
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg        enable;
    reg        invert;
    reg [15:0] prescale;
    reg [31:0] period;

    reg [31:0] duty [0:CHANNELS-1];
    reg [CHANNELS-1:0] ch_enable;

    wire [31:0] count;

    timer_core PWM_COUNTER (
        .clk        (clk),
        .reset      (reset),
        .enable     (enable),
        .clear      (1'b0),
        .auto_reload(1'b1),
        .prescale   (prescale),
        .period     (period),
        .count      (count),
        .tick       (),
        .pre_tick   ()
    );

    integer k;
    always @(posedge clk) begin
        if (reset) begin
            enable    <= 1'b0;
            invert    <= 1'b0;
            prescale  <= 16'd0;
            period    <= 32'd999;
            ch_enable <= {CHANNELS{1'b0}};
            for (k = 0; k < CHANNELS; k = k + 1)
                duty[k] <= 32'd0;
        end
        else if (wr) begin
            case (reg_index)
                6'h00: begin
                    enable <= wdata[0];
                    invert <= wdata[1];
                end
                6'h01: prescale <= wdata[15:0];
                6'h02: period   <= wdata;
                6'h04: if (CHANNELS > 0) duty[0] <= wdata;
                6'h05: if (CHANNELS > 1) duty[1] <= wdata;
                6'h06: if (CHANNELS > 2) duty[2] <= wdata;
                6'h07: if (CHANNELS > 3) duty[3] <= wdata;
                6'h08: ch_enable <= wdata[CHANNELS-1:0];
                default: ;
            endcase
        end
    end

    // Comparator per channel, registered so the output is glitch free.
    integer c;
    always @(posedge clk) begin
        if (reset) begin
            pwm_out <= {CHANNELS{1'b0}};
        end
        else begin
            for (c = 0; c < CHANNELS; c = c + 1) begin
                if (!enable || !ch_enable[c])
                    pwm_out[c] <= invert;
                else
                    pwm_out[c] <= (count < duty[c]) ? ~invert : invert;
            end
        end
    end

    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {30'b0, invert, enable};
                6'h01:   prdata <= {16'b0, prescale};
                6'h02:   prdata <= period;
                6'h03:   prdata <= count;
                6'h04:   prdata <= duty[0];
                6'h05:   prdata <= duty[1];
                6'h06:   prdata <= duty[2];
                6'h07:   prdata <= duty[3];
                6'h08:   prdata <= {{(32-CHANNELS){1'b0}}, ch_enable};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
