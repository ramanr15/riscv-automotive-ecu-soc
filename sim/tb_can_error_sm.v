`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_error_sm.v
//
// Step 8 self-check, per Section 2.3.11 item 8 / Person 3's own
// verification plan for this block: "directly drive a sequence of
// injected errors and confirm TEC/REC increment/decrement
// according to the documented rules, and that the Error-Active to
// Error-Passive to Bus-Off transitions occur at the correct
// thresholds (128 and 255); confirm the bus-off recovery sequence
// (128 occurrences of 11 consecutive recessive bits) correctly
// returns the controller to Error-Active."
// ============================================================

module tb_can_error_sm;

    reg clk, rst_n, can_enable;
    reg frame_good, frame_error, is_transmitter;
    reg sample_point, bus_bit;

    wire [7:0] tec, rec;
    wire [1:0] err_state;
    wire       err_state_changed;

    can_error_sm DUT (
        .clk(clk), .rst_n(rst_n), .can_enable(can_enable),
        .frame_good(frame_good), .frame_error(frame_error),
        .is_transmitter(is_transmitter),
        .sample_point(sample_point), .bus_bit(bus_bit),
        .tec(tec), .rec(rec),
        .err_state(err_state), .err_state_changed(err_state_changed)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;

    task check;
        input cond;
        input [255:0] name;
        begin
            if (!cond) begin
                $display("FAIL: %0s @ time %0t (tec=%0d rec=%0d err_state=%0d err_state_changed=%0b)",
                          name, $time, tec, rec, err_state, err_state_changed);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s @ time %0t", name, $time);
            end
        end
    endtask

    task pulse_error;
        input tx;
        begin
            @(negedge clk);
            is_transmitter = tx;
            frame_error = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            frame_error = 1'b0;
        end
    endtask

    task pulse_good;
        input tx;
        begin
            @(negedge clk);
            is_transmitter = tx;
            frame_good = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            frame_good = 1'b0;
        end
    endtask

    task do_sample;
        input bb;
        begin
            @(negedge clk);
            bus_bit = bb;
            sample_point = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            sample_point = 1'b0;
        end
    endtask

    // One completed occurrence: 11 consecutive recessive samples.
    task recessive_run_11;
        integer i;
        begin
            for (i = 0; i < 11; i = i + 1)
                do_sample(`CAN_RECESSIVE);
        end
    endtask

    task settle;
        begin
            @(posedge clk); #1;
        end
    endtask

    task reinit;
        begin
            @(negedge clk);
            can_enable = 1'b0;
            @(posedge clk); #1;
            @(negedge clk);
            can_enable = 1'b1;
        end
    endtask

    integer k;

    initial begin
        errors = 0;
        rst_n = 1'b0; can_enable = 1'b0;
        frame_good = 1'b0; frame_error = 1'b0; is_transmitter = 1'b0;
        sample_point = 1'b0; bus_bit = 1'b1;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        can_enable = 1'b1;
        repeat (2) @(posedge clk);

        check(err_state === `CAN_ERR_ACTIVE, "reset_state_is_active");
        check(tec === 8'd0, "reset_tec_zero");
        check(rec === 8'd0, "reset_rec_zero");

        // ============================================================
        // Scenario A: a TX error increases TEC by 8; RC is untouched.
        // ============================================================
        pulse_error(1'b1);
        check(tec === 8'd8, "A_tx_error_tec_plus_8");
        check(rec === 8'd0, "A_tx_error_rec_untouched");

        // ============================================================
        // Scenario B: an RX error increases REC by 8; TEC untouched.
        // ============================================================
        pulse_error(1'b0);
        check(rec === 8'd8, "B_rx_error_rec_plus_8");
        check(tec === 8'd8, "B_rx_error_tec_untouched");

        // ============================================================
        // Scenario C: a clean TX frame decreases TEC by 1; a clean
        // RX frame decreases REC by 1.
        // ============================================================
        pulse_good(1'b1);
        check(tec === 8'd7, "C_tx_good_tec_minus_1");
        pulse_good(1'b0);
        check(rec === 8'd7, "C_rx_good_rec_minus_1");

        // ============================================================
        // Scenario D: decrement floors at 0, never wraps negative.
        // ============================================================
        reinit;
        check(tec === 8'd0, "D_reinit_tec_zero");
        pulse_good(1'b1);
        check(tec === 8'd0, "D_good_at_zero_stays_zero");
        pulse_good(1'b0);
        check(rec === 8'd0, "D_rx_good_at_zero_stays_zero");

        // ============================================================
        // Scenario E: Error-Active -> Error-Passive at TEC == 128.
        // err_state lags the counter by exactly one clock (see
        // can_error_sm.v header) - checked explicitly both ways.
        // ============================================================
        reinit;
        for (k = 0; k < 16; k = k + 1) // 16 * 8 = 128
            pulse_error(1'b1);

        check(tec === 8'd128, "E_tec_reaches_128");
        check(err_state === `CAN_ERR_ACTIVE, "E_state_not_yet_updated_same_cycle");
        settle;
        check(err_state === `CAN_ERR_PASSIVE, "E_state_becomes_passive_after_settle");
        check(err_state_changed === 1'b1, "E_state_changed_pulses_once");
        settle;
        check(err_state_changed === 1'b0, "E_state_changed_is_one_shot");

        // ============================================================
        // Scenario F: Error-Passive -> Bus-Off when TEC exceeds 255.
        // ============================================================
        for (k = 0; k < 16; k = k + 1) // 16 more * 8 = 128 -> tec_cnt = 256
            pulse_error(1'b1);

        check(tec === 8'd255, "F_tec_display_saturates_at_255");
        check(err_state === `CAN_ERR_PASSIVE, "F_state_not_yet_updated_same_cycle");
        settle;
        check(err_state === `CAN_ERR_BUSOFF, "F_state_becomes_busoff_after_settle");
        check(err_state_changed === 1'b1, "F_busoff_state_changed_pulses_once");

        // ============================================================
        // Scenario G: while Bus-Off, ordinary frame_error/frame_good
        // pulses must NOT move the counters (transmission is halted
        // in real hardware; this is a defensive RTL check).
        // ============================================================
        pulse_error(1'b1);
        check(tec === 8'd255, "G_busoff_freezes_tec_on_error");
        pulse_good(1'b1);
        check(tec === 8'd255, "G_busoff_freezes_tec_on_good");

        // ============================================================
        // Scenario H: Bus-Off recovery - 128 occurrences of 11
        // consecutive recessive bits. A dominant bit partway through
        // one attempt interrupts that run without undoing the
        // occurrences already completed.
        // ============================================================
        for (k = 0; k < 64; k = k + 1)
            recessive_run_11; // 64 completed occurrences

        check(err_state === `CAN_ERR_BUSOFF, "H_still_busoff_at_64_occurrences");

        // Interrupted attempt: 5 recessive bits, then a dominant bit.
        do_sample(`CAN_RECESSIVE);
        do_sample(`CAN_RECESSIVE);
        do_sample(`CAN_RECESSIVE);
        do_sample(`CAN_RECESSIVE);
        do_sample(`CAN_RECESSIVE);
        do_sample(`CAN_DOMINANT); // interrupts - this attempt does NOT count
        check(err_state === `CAN_ERR_BUSOFF, "H_still_busoff_after_interrupted_run");

        // Redo that run cleanly (occurrence #65), then the remaining
        // 63 to reach 128 total.
        for (k = 0; k < 64; k = k + 1)
            recessive_run_11; // 64 more -> 128 total completed occurrences

        check(tec === 8'd0, "H_recovery_clears_tec");
        check(rec === 8'd0, "H_recovery_clears_rec");
        check(err_state === `CAN_ERR_BUSOFF, "H_state_not_yet_updated_same_cycle");
        settle;
        check(err_state === `CAN_ERR_ACTIVE, "H_recovery_returns_to_active");
        check(err_state_changed === 1'b1, "H_recovery_state_changed_pulses");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_error_sm: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_error_sm: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
