`timescale 1ns / 1ps
// ============================================================
// timer_core : Standalone reusable up-counter with prescaler
// Person 2 - Step 8 (also the base for the PWM in Step 9)
//
// The prescaler divides the system clock before the main
// counter increments, so long periods are reachable without a
// very wide counter.
//   effective tick rate = f_clk / (prescale + 1)
//   period in ticks     = period + 1
// ============================================================
module timer_core (
    input  wire        clk,
    input  wire        reset,

    input  wire        enable,
    input  wire        clear,
    input  wire        auto_reload,

    input  wire [15:0] prescale,
    input  wire [31:0] period,

    output reg  [31:0] count,
    output reg         tick,       // 1-cycle pulse on count == period
    output reg         pre_tick    // 1-cycle pulse each prescaler expiry
);

    reg [15:0] pre_cnt;

    always @(posedge clk) begin
        if (reset || clear || !enable) begin
            pre_cnt  <= 16'd0;
            pre_tick <= 1'b0;
        end
        else if (pre_cnt >= prescale) begin
            pre_cnt  <= 16'd0;
            pre_tick <= 1'b1;
        end
        else begin
            pre_cnt  <= pre_cnt + 16'd1;
            pre_tick <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (reset || clear) begin
            count <= 32'd0;
            tick  <= 1'b0;
        end
        else begin
            tick <= 1'b0;

            if (enable && pre_tick) begin
                if (count >= period) begin
                    tick  <= 1'b1;
                    count <= auto_reload ? 32'd0 : (count + 32'd1);
                end
                else begin
                    count <= count + 32'd1;
                end
            end
        end
    end

endmodule
