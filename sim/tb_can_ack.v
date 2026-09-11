`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_ack.v
//
// Step 7 self-check, per Section 2.3.11 item 7 / Person 3's own
// verification plan for this block: "test normal acknowledgment,
// then force the ACK slot to remain recessive and confirm an ACK
// error is raised."
//
// Also exercises: ack_error held across cycles (not a one-shot
// pulse, per the frame FSM's timing contract), reset_ack starting
// a fresh frame, the receiver role's ack_drive (unconditionally
// dominant - see can_ack.v's header note on why there's no
// separate crc_ok input), and the ack_enable scope boundary.
// ============================================================

module tb_can_ack;

    reg clk, rst_n;
    reg reset_ack, ack_enable, sample_point;
    reg is_transmitter, bus_bit;

    wire ack_drive, ack_error;

    can_ack DUT (
        .clk(clk), .rst_n(rst_n),
        .reset_ack(reset_ack),
        .ack_enable(ack_enable),
        .sample_point(sample_point),
        .is_transmitter(is_transmitter),
        .bus_bit(bus_bit),
        .ack_drive(ack_drive),
        .ack_error(ack_error)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;

    task check;
        input cond;
        input [255:0] name;
        begin
            if (!cond) begin
                $display("FAIL: %0s @ time %0t (ack_drive=%0b ack_error=%0b)",
                          name, $time, ack_drive, ack_error);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s @ time %0t", name, $time);
            end
        end
    endtask

    task tick;
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

    task pulse_reset_ack;
        begin
            @(negedge clk);
            reset_ack = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            reset_ack = 1'b0;
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0;
        reset_ack = 1'b0; ack_enable = 1'b0; sample_point = 1'b0;
        is_transmitter = 1'b0; bus_bit = 1'b1;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        check(ack_error === 1'b0, "reset_ack_error_clear");

        // ============================================================
        // Scenario A: transmitter, normal acknowledgment - drives
        // recessive, samples a dominant bus -> no error.
        // ============================================================
        pulse_reset_ack;
        is_transmitter = 1'b1;
        ack_enable = 1'b1;
        #1; // let the combinational ack_drive settle before reading it

        check(ack_drive === `CAN_RECESSIVE, "A_transmitter_drives_recessive");

        tick(`CAN_DOMINANT); // some receiver pulled it dominant
        check(ack_error === 1'b0, "A_acked_no_error");

        ack_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario B: transmitter, nobody acks - bus stays recessive
        // through the ACK slot -> ack_error raised.
        // ============================================================
        pulse_reset_ack;
        is_transmitter = 1'b1;
        ack_enable = 1'b1;

        tick(`CAN_RECESSIVE); // nobody pulled it dominant
        check(ack_error === 1'b1, "B_no_ack_raises_ack_error");

        // ============================================================
        // Scenario C: ack_error must stay held (not a one-shot
        // pulse) across further clocks with no new sample_point,
        // since the frame FSM's exit sync_pulse for CAN_FLD_ACK_SLOT
        // may land several clocks after the sample_point that set it.
        // ============================================================
        ack_enable = 1'b0; // field has moved on to ACK_DELIM
        repeat (4) @(posedge clk);
        check(ack_error === 1'b1, "C_ack_error_held_after_field_moves_on");

        // ============================================================
        // Scenario D: reset_ack (next frame's SOF) clears it; a
        // normal ack in the new frame keeps it clear.
        // ============================================================
        pulse_reset_ack;
        check(ack_error === 1'b0, "D_reset_ack_clears_error");

        ack_enable = 1'b1;
        tick(`CAN_DOMINANT);
        check(ack_error === 1'b0, "D_fresh_frame_normal_ack_stays_clear");

        ack_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario E: receiver role - always drives dominant
        // (reaching ACK_SLOT already implies this node's own CRC
        // passed - see can_ack.v header), and never raises
        // ack_error itself regardless of what the bus reads.
        // ============================================================
        pulse_reset_ack;
        is_transmitter = 1'b0;
        ack_enable = 1'b1;
        #1; // let the combinational ack_drive settle before reading it

        check(ack_drive === `CAN_DOMINANT, "E_receiver_drives_dominant");

        tick(`CAN_RECESSIVE); // even a "no other ack" bus reading...
        check(ack_error === 1'b0, "E_receiver_never_raises_ack_error");
        tick(`CAN_DOMINANT);
        check(ack_error === 1'b0, "E_receiver_still_no_ack_error");

        ack_enable = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Scenario F: scope check - a sample_point with a recessive
        // bus while ack_enable=0 (outside the ACK Slot field) must
        // not raise ack_error, even as a transmitter.
        // ============================================================
        pulse_reset_ack;
        is_transmitter = 1'b1;
        ack_enable = 1'b0;

        tick(`CAN_RECESSIVE);
        check(ack_error === 1'b0, "F_outside_ack_slot_no_error");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_ack: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_ack: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
