`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_arbitration.v
//
// Step 6 self-check, per Section 2.3.11 item 6: "Arbitration
// logic, tested by simulating two competing IDs and confirming
// the lower ID wins."
//
// This is a single-node unit test (matching the rest of this
// project's per-block testbenches): rather than instantiating two
// full nodes, it drives `my_bit` (this node's own driven bit) and
// `bus_bit` (the monitored bus level, i.e. what a second,
// higher-priority - lower ID - competitor would force the bus to)
// directly, which is exactly the interface can_arbitration.v
// reacts to either way.
//
// Scenarios:
//   A. Tied/winning competitor - my_bit == bus_bit every bit of
//      the Arbitration field -> arbitration_lost never fires.
//   B. Losing competitor - a bit where my_bit is recessive but
//      bus_bit is dominant -> arbitration_lost pulses exactly once.
//   C. After losing, further mismatched bits in the same frame
//      must NOT re-pulse (latched for the rest of the frame).
//   D. reset_arb (next frame's SOF) clears the latch so a genuine
//      loss in the new frame is detected again.
//   E. Scope check - a mismatch outside the Arbitration field
//      (arb_enable=0) must never assert arbitration_lost.
//   F. A mismatch presented without a sample_point pulse must not
//      assert arbitration_lost.
// ============================================================

module tb_can_arbitration;

    reg clk, rst_n;
    reg reset_arb, arb_enable, sample_point;
    reg my_bit, bus_bit;

    wire arbitration_lost, lost_latched;

    can_arbitration DUT (
        .clk(clk), .rst_n(rst_n),
        .reset_arb(reset_arb),
        .arb_enable(arb_enable),
        .sample_point(sample_point),
        .my_bit(my_bit), .bus_bit(bus_bit),
        .arbitration_lost(arbitration_lost),
        .lost_latched(lost_latched)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;

    task check;
        input cond;
        input [255:0] name;
        begin
            if (!cond) begin
                $display("FAIL: %0s @ time %0t (arbitration_lost=%0b lost_latched=%0b)",
                          name, $time, arbitration_lost, lost_latched);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s @ time %0t", name, $time);
            end
        end
    endtask

    // Present one bit-time: drive my_bit/bus_bit and pulse
    // sample_point for one clock, then settle.
    task tick;
        input mb, bb;
        begin
            @(negedge clk);
            my_bit = mb;
            bus_bit = bb;
            sample_point = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            sample_point = 1'b0;
        end
    endtask

    // Present a bit-time with NO sample_point pulse at all (levels
    // change, but nothing should be sampled) - for scenario F.
    task tick_no_sample;
        input mb, bb;
        begin
            @(negedge clk);
            my_bit = mb;
            bus_bit = bb;
            @(posedge clk); #1;
            @(negedge clk);
        end
    endtask

    task pulse_reset_arb;
        begin
            @(negedge clk);
            reset_arb = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            reset_arb = 1'b0;
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0;
        reset_arb = 1'b0; arb_enable = 1'b0; sample_point = 1'b0;
        my_bit = 1'b1; bus_bit = 1'b1;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        check(arbitration_lost === 1'b0, "reset_arbitration_lost_clear");
        check(lost_latched     === 1'b0, "reset_lost_latched_clear");

        // ============================================================
        // Scenario A: tied/winning competitor - my_bit tracks bus_bit
        // exactly through a full 12-bit Arbitration field (11-bit ID +
        // RTR). Values themselves don't matter, only that they match.
        // ============================================================
        pulse_reset_arb;
        arb_enable = 1'b1;

        tick(1'b0, 1'b0); tick(1'b0, 1'b0); tick(1'b1, 1'b1); tick(1'b0, 1'b0);
        tick(1'b1, 1'b1); tick(1'b1, 1'b1); tick(1'b0, 1'b0); tick(1'b1, 1'b1);
        tick(1'b0, 1'b0); tick(1'b1, 1'b1); tick(1'b1, 1'b1); tick(1'b0, 1'b0);

        check(arbitration_lost === 1'b0, "A_never_lost_when_tied");
        check(lost_latched     === 1'b0, "A_latch_stays_clear");

        arb_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario B: losing competitor. Ties for the first few bits,
        // then this node drives recessive while the bus reads
        // dominant (another node has a lower/higher-priority ID) -
        // arbitration_lost must pulse exactly on that bit.
        // ============================================================
        pulse_reset_arb;
        arb_enable = 1'b1;

        tick(1'b0, 1'b0); // tie (dominant)
        tick(1'b0, 1'b0); // tie (dominant)
        tick(1'b1, 1'b1); // tie (recessive)

        tick(1'b1, 1'b0); // MISMATCH: we drive recessive, bus is dominant
        check(arbitration_lost === 1'b1, "B_arbitration_lost_pulses");
        check(lost_latched     === 1'b1, "B_latch_sets");

        // ============================================================
        // Scenario C: further bits in the same frame, including
        // another bit that would (in isolation) also be a loss -
        // must NOT re-pulse, since the latch is already set.
        // ============================================================
        tick(1'b1, 1'b0); // same mismatch pattern again
        check(arbitration_lost === 1'b0, "C_no_repulse_after_latched");
        check(lost_latched     === 1'b1, "C_latch_still_set");

        tick(1'b0, 1'b0); // even a tie afterward shouldn't clear/pulse
        check(arbitration_lost === 1'b0, "C_tie_after_loss_no_pulse");
        check(lost_latched     === 1'b1, "C_latch_still_set_after_tie");

        arb_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario D: reset_arb (next frame's SOF) clears the latch;
        // a genuine loss in the new frame is detected again.
        // ============================================================
        pulse_reset_arb;
        check(lost_latched === 1'b0, "D_reset_arb_clears_latch");

        arb_enable = 1'b1;
        tick(1'b0, 1'b0); // tie
        tick(1'b1, 1'b0); // MISMATCH again, fresh frame
        check(arbitration_lost === 1'b1, "D_fresh_frame_detects_loss_again");
        check(lost_latched     === 1'b1, "D_fresh_frame_latch_sets");

        arb_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario E: scope check - a mismatch outside the
        // Arbitration field (arb_enable=0) must never assert
        // arbitration_lost, even at a real sample_point.
        // ============================================================
        pulse_reset_arb;
        arb_enable = 1'b0; // e.g. Control/Data field - not arbitration

        tick(1'b1, 1'b0); // would be a mismatch if arb_enable were 1
        check(arbitration_lost === 1'b0, "E_outside_arb_field_no_pulse");
        check(lost_latched     === 1'b0, "E_outside_arb_field_no_latch");

        // ============================================================
        // Scenario F: a mismatch presented with no sample_point pulse
        // must not be sampled at all.
        // ============================================================
        pulse_reset_arb;
        arb_enable = 1'b1;

        tick_no_sample(1'b1, 1'b0); // mismatched levels, but never sampled
        check(arbitration_lost === 1'b0, "F_no_sample_point_no_pulse");
        check(lost_latched     === 1'b0, "F_no_sample_point_no_latch");

        // Confirm the module still works normally right afterward.
        tick(1'b1, 1'b0);
        check(arbitration_lost === 1'b1, "F_real_sample_still_detects_loss");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_arbitration: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_arbitration: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
