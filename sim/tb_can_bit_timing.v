`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_bit_timing.v
//
// Self-checking bench for Step 1 (can_bit_timing.v), per the
// WBS's own test description:
//   "Force a known BRP/segment configuration, toggle the input
//    bus line, and check in simulation that sample_point pulses
//    land exactly where hand-calculated bit timing predicts."
//
// Run with, e.g.:
//   iverilog -o sim.out -I ../rtl ../rtl/can_bit_timing.v tb_can_bit_timing.v
//   vvp sim.out
// (or add both files to a Vivado behavioral simulation with this
// as the simulation top, matching the base core's XSim flow).
// ============================================================

module tb_can_bit_timing;

    reg clk;
    reg rst_n;
    reg can_enable;

    reg [7:0] brp;
    reg [3:0] prop_seg;
    reg [3:0] phase_seg1;
    reg [3:0] phase_seg2;
    reg [1:0] sjw;

    reg hard_sync;
    reg resync_edge;

    wire tq_pulse;
    wire sample_point;
    wire sync_pulse;
    wire [2:0] seg_state;

    integer errors;

    can_bit_timing DUT (
        .clk          (clk),
        .rst_n        (rst_n),
        .can_enable   (can_enable),
        .brp          (brp),
        .prop_seg     (prop_seg),
        .phase_seg1   (phase_seg1),
        .phase_seg2   (phase_seg2),
        .sjw          (sjw),
        .hard_sync    (hard_sync),
        .resync_edge  (resync_edge),
        .tq_pulse     (tq_pulse),
        .sample_point (sample_point),
        .sync_pulse   (sync_pulse),
        .seg_state    (seg_state)
    );

    // 100 MHz system clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_equal;
        input [63:0] actual;
        input [63:0] expected;
        input [255:0] name;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s = %0d, expected %0d @ time %0t",
                          name, actual, expected, $time);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s = %0d @ time %0t", name, actual, $time);
            end
        end
    endtask

    // ------------------------------------------------------------
    // Measure clocks between two consecutive tq_pulses and check
    // it equals brp + 1.
    // ------------------------------------------------------------
    task check_clocks_per_tq;
        integer t0, t1;
        begin
            @(posedge tq_pulse);
            t0 = $time;
            @(posedge tq_pulse);
            t1 = $time;
            check_equal((t1 - t0) / 10, brp + 1, "clocks_between_tq_pulses");
        end
    endtask

    // ------------------------------------------------------------
    // Count tq_pulses from "now" until the next sample_point and
    // check it equals the expected TQ offset.
    // ------------------------------------------------------------
    task count_tq_until_sample;
        input [63:0] expected_tq;
        input [255:0] name;
        integer n;
        reg     done;
        begin
            n    = 0;
            done = 1'b0;
            while (!done) begin
                @(posedge clk);
                if (sample_point) begin
                    check_equal(n, expected_tq, name);
                    done = 1'b1;
                end
                else if (tq_pulse) begin
                    n = n + 1;
                end
            end
        end
    endtask

    initial begin
        errors      = 0;
        rst_n       = 1'b0;
        can_enable  = 1'b0;
        hard_sync   = 1'b0;
        resync_edge = 1'b0;

        // brp=3  -> 4 clocks per time quantum
        // prop=2, phase1=3, phase2=2, sjw=1
        // -> bit length = 1(sync)+2+3+2 = 8 TQ = 32 clocks
        // -> sample point should land prop+phase1 = 5 TQ after sync_pulse
        //    (sync_pulse itself already marks the end of SYNC_SEG's TQ)
        brp        = 8'd3;
        prop_seg   = 4'd2;
        phase_seg1 = 4'd3;
        phase_seg2 = 4'd2;
        sjw        = 2'd1;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        can_enable = 1'b1;

        // Check 1: clock-domain prescale ratio.
        check_clocks_per_tq;

        // Check 2: TQ pulses from the SYNC/PROP boundary to the next
        // sample_point. sync_pulse fires *after* SYNC_SEG's own 1 TQ
        // has already elapsed (same trailing-boundary convention as
        // sample_point itself, see can_bit_timing.v's port comment),
        // so the remaining distance to the PHASE1/PHASE2 boundary is
        // just prop_seg + phase_seg1 (SYNC_SEG's TQ is not counted
        // again here - it already elapsed before sync_pulse fired).
        @(posedge sync_pulse);
        count_tq_until_sample(prop_seg + phase_seg1,
                               "tq_sync_to_sample_cfg1");

        // Check 3: a second, different configuration - confirms
        // the generator reacts to the register fields rather than
        // being hard-coded for the first test.
        can_enable = 1'b0;
        repeat (2) @(posedge clk);
        brp        = 8'd1;   // 2 clocks per TQ
        prop_seg   = 4'd1;
        phase_seg1 = 4'd2;
        phase_seg2 = 4'd1;
        sjw        = 2'd1;
        repeat (2) @(posedge clk);
        can_enable = 1'b1;

        @(posedge sync_pulse);
        count_tq_until_sample(prop_seg + phase_seg1,
                               "tq_sync_to_sample_cfg2");

        repeat (20) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_bit_timing: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_bit_timing: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
