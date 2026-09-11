`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_frame_fsm.v
//
// Step 5 self-check, per Section 2.3.11 item 4: "Frame field state
// machine, driven by a fixed DLC first, then a variable DLC."
//
// Also exercises: DLC=0 (Data field must be skipped entirely, not
// just zero-length), arbitration_lost mid-Arbitration field, an
// injected stuff_error aborting into the error field, and a basic
// receive-path walk using rx_dlc_capture instead of tx_dlc.
// ============================================================

module tb_can_frame_fsm;

    reg clk, rst_n, can_enable;
    reg sync_pulse;
    reg stuff_hold;
    reg tx_request;
    reg [3:0] tx_dlc;
    reg rx_start;
    reg [3:0] rx_dlc_capture;
    reg arbitration_lost, crc_error, ack_error, stuff_error, bit_error;

    wire [3:0] field_state;
    wire [6:0] field_bit_index;
    wire [6:0] field_len;
    wire is_transmitter;
    wire stuff_enable, crc_enable;
    wire reset_stuff, reset_crc, reset_arb, reset_ack;
    wire frame_good, frame_error, go_idle;

    integer errors;

    can_frame_fsm DUT (
        .clk(clk), .rst_n(rst_n), .can_enable(can_enable),
        .sync_pulse(sync_pulse), .stuff_hold(stuff_hold),
        .tx_request(tx_request), .tx_dlc(tx_dlc), .rx_start(rx_start),
        .rx_dlc_capture(rx_dlc_capture),
        .arbitration_lost(arbitration_lost),
        .crc_error(crc_error), .ack_error(ack_error),
        .stuff_error(stuff_error), .bit_error(bit_error),
        .field_state(field_state), .field_bit_index(field_bit_index),
        .field_len(field_len), .is_transmitter(is_transmitter),
        .stuff_enable(stuff_enable), .crc_enable(crc_enable),
        .reset_stuff(reset_stuff), .reset_crc(reset_crc), .reset_arb(reset_arb),
        .reset_ack(reset_ack),
        .frame_good(frame_good), .frame_error(frame_error), .go_idle(go_idle)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check;
        input cond;
        input [255:0] name;
        begin
            if (!cond) begin
                $display("FAIL: %0s @ time %0t (field_state=%0d field_len=%0d field_bit_index=%0d)",
                          name, $time, field_state, field_len, field_bit_index);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s @ time %0t", name, $time);
            end
        end
    endtask

    task tick;
        begin
            @(negedge clk);
            sync_pulse = 1'b1;
            @(negedge clk);
            sync_pulse = 1'b0;
        end
    endtask

    // Like tick, but with stuff_hold asserted - simulates the wire
    // bit-time that just completed having been a stuff bit.
    task tick_hold;
        begin
            @(negedge clk);
            stuff_hold = 1'b1;
            sync_pulse = 1'b1;
            @(negedge clk);
            stuff_hold = 1'b0;
            sync_pulse = 1'b0;
        end
    endtask

    // Tick until field_state == target (or a safety guard trips),
    // then check the field_len it arrived with.
    task advance_to;
        input [3:0] target_state;
        input [6:0] expected_len;
        input [255:0] name;
        integer guard;
        begin
            guard = 0;
            while ((field_state !== target_state) && (guard < 300)) begin
                tick;
                guard = guard + 1;
            end
            check(field_state === target_state, {name, "_reached"});
            check(field_len === expected_len, {name, "_len"});
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0; can_enable = 1'b0; sync_pulse = 1'b0; stuff_hold = 1'b0;
        tx_request = 1'b0; tx_dlc = 4'd0; rx_start = 1'b0; rx_dlc_capture = 4'd0;
        arbitration_lost = 1'b0; crc_error = 1'b0; ack_error = 1'b0;
        stuff_error = 1'b0; bit_error = 1'b0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        can_enable = 1'b1;

        check(field_state === `CAN_FLD_IDLE, "reset_state_is_idle");

        // ============================================================
        // Test A: TX path, fixed DLC = 8 (full 64-bit data field)
        // ============================================================
        tx_dlc = 4'd8;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "A_sof");
        check(reset_stuff === 1'b1, "A_reset_stuff_pulses_at_sof");
        check(reset_crc   === 1'b1, "A_reset_crc_pulses_at_sof");
        check(reset_arb   === 1'b1, "A_reset_arb_pulses_at_sof");
        check(reset_ack   === 1'b1, "A_reset_ack_pulses_at_sof");
        check(is_transmitter === 1'b1, "A_is_transmitter_set");

        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "A_arb");
        advance_to(`CAN_FLD_CONTROL,     7'd6,  "A_ctrl");
        advance_to(`CAN_FLD_DATA,        7'd64, "A_data_dlc8");
        advance_to(`CAN_FLD_CRC,         7'd15, "A_crc");
        advance_to(`CAN_FLD_CRC_DELIM,   7'd1,  "A_crc_delim");
        advance_to(`CAN_FLD_ACK_SLOT,    7'd1,  "A_ack_slot");
        advance_to(`CAN_FLD_ACK_DELIM,   7'd1,  "A_ack_delim");
        advance_to(`CAN_FLD_EOF,         7'd7,  "A_eof");
        advance_to(`CAN_FLD_INTERMISSION,7'd3,  "A_intermission");
        check(frame_good === 1'b1, "A_frame_good_pulses_at_eof_done");

        advance_to(`CAN_FLD_IDLE, 7'd0, "A_back_to_idle");
        check(go_idle === 1'b1, "A_go_idle_pulses");
        check(is_transmitter === 1'b1, "A_stayed_transmitter_whole_frame");

        tx_request = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test B: TX path, DLC = 0 - the Data field must be skipped
        // entirely (not merely zero-length), i.e. CONTROL's last bit
        // must transition directly to CRC.
        // ============================================================
        tx_dlc = 4'd0;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "B_sof");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "B_arb");
        advance_to(`CAN_FLD_CONTROL, 7'd6, "B_ctrl");

        repeat (5) tick; // consume 5 of CONTROL's 6 bits
        check(field_state === `CAN_FLD_CONTROL, "B_still_in_control_before_last_bit");

        tick; // the exit tick
        check(field_state === `CAN_FLD_CRC, "B_dlc0_skips_data_goes_directly_to_crc");
        check(field_len   === 7'd15,        "B_dlc0_crc_len");

        advance_to(`CAN_FLD_CRC_DELIM,    7'd1, "B_crc_delim");
        advance_to(`CAN_FLD_ACK_SLOT,     7'd1, "B_ack_slot");
        advance_to(`CAN_FLD_ACK_DELIM,    7'd1, "B_ack_delim");
        advance_to(`CAN_FLD_EOF,          7'd7, "B_eof");
        advance_to(`CAN_FLD_INTERMISSION, 7'd3, "B_intermission");
        advance_to(`CAN_FLD_IDLE,         7'd0, "B_back_to_idle");

        tx_request = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test C: TX path, a non-zero/non-8 DLC (variable-DLC check)
        // ============================================================
        tx_dlc = 4'd3;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "C_sof");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "C_arb");
        advance_to(`CAN_FLD_CONTROL, 7'd6, "C_ctrl");
        advance_to(`CAN_FLD_DATA, 7'd24, "C_data_dlc3"); // 3 bytes = 24 bits

        // Drain out the rest of the frame without further checks.
        advance_to(`CAN_FLD_IDLE, 7'd0, "C_back_to_idle");
        tx_request = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test D: losing arbitration mid-field - is_transmitter must
        // clear, but field boundaries must be unaffected (everyone
        // stays synchronized to the same bit stream).
        // ============================================================
        tx_dlc = 4'd1;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "D_sof");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "D_arb");
        repeat (3) tick; // partway into the Arbitration field
        check(is_transmitter === 1'b1, "D_still_transmitter_before_loss");

        @(negedge clk);
        arbitration_lost = 1'b1;
        sync_pulse = 1'b1;
        @(negedge clk);
        arbitration_lost = 1'b0;
        sync_pulse = 1'b0;

        check(is_transmitter === 1'b0, "D_lost_arbitration_clears_is_transmitter");
        check(field_state === `CAN_FLD_ARBITRATION, "D_still_in_arbitration_field");

        // Field sequencing continues normally despite the loss.
        advance_to(`CAN_FLD_CONTROL, 7'd6, "D_ctrl_after_loss");
        advance_to(`CAN_FLD_IDLE, 7'd0, "D_back_to_idle");
        check(is_transmitter === 1'b0, "D_stayed_receiver_rest_of_frame");
        tx_request = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test E: an injected stuff_error aborts into the error field
        // from any point in the frame, then recovers to IDLE.
        // ============================================================
        tx_dlc = 4'd8;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "E_sof");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "E_arb");
        advance_to(`CAN_FLD_CONTROL, 7'd6, "E_ctrl");
        advance_to(`CAN_FLD_DATA, 7'd64, "E_data");
        repeat (5) tick; // partway into Data

        @(negedge clk);
        stuff_error = 1'b1;
        sync_pulse = 1'b1;
        @(negedge clk);
        stuff_error = 1'b0;
        sync_pulse = 1'b0;

        check(field_state === `CAN_FLD_ERROR, "E_aborts_to_error_field");
        check(frame_error === 1'b1, "E_frame_error_pulses");

        advance_to(`CAN_FLD_INTERMISSION, 7'd3, "E_intermission_after_error");
        advance_to(`CAN_FLD_IDLE, 7'd0, "E_recovers_to_idle");
        tx_request = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test F: receive path - rx_start (not tx_request), DLC comes
        // from rx_dlc_capture instead of tx_dlc.
        // ============================================================
        rx_dlc_capture = 4'd5; // 5 bytes = 40 bits
        rx_start = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "F_sof");
        check(is_transmitter === 1'b0, "F_receiver_not_transmitter");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "F_arb");
        advance_to(`CAN_FLD_CONTROL, 7'd6, "F_ctrl");
        advance_to(`CAN_FLD_DATA, 7'd40, "F_data_from_rx_dlc_capture");
        advance_to(`CAN_FLD_IDLE, 7'd0, "F_back_to_idle");
        rx_start = 1'b0;
        repeat (3) tick;

        // ============================================================
        // Test G: stuff_hold - a sync_pulse presented with stuff_hold
        // high must NOT advance field_state/field_bit_index at all
        // (the wire bit-time that just completed was a stuff bit,
        // not a logical content bit); the next ordinary tick then
        // advances normally, as if the held tick never happened.
        // ============================================================
        tx_dlc = 4'd8;
        tx_request = 1'b1;

        advance_to(`CAN_FLD_SOF, 7'd1, "G_sof");
        advance_to(`CAN_FLD_ARBITRATION, 7'd12, "G_arb");
        repeat (3) tick; // partway into Arbitration
        check(field_bit_index === 7'd3, "G_index_before_hold");

        tick_hold;
        check(field_state === `CAN_FLD_ARBITRATION, "G_hold_keeps_field");
        check(field_bit_index === 7'd3, "G_hold_keeps_index");

        tick; // ordinary tick resumes normal advancement
        check(field_bit_index === 7'd4, "G_resumes_after_hold");

        advance_to(`CAN_FLD_IDLE, 7'd0, "G_back_to_idle");
        tx_request = 1'b0;
        repeat (3) tick;

        repeat (10) tick;

        if (errors == 0)
            $display("\n=== tb_can_frame_fsm: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_frame_fsm: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
