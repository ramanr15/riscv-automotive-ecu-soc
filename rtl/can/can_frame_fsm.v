`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_frame_fsm.v
//
// WBS Section 2.3.6 - Step 5: Frame Field State Machine
//
// Walks through the fields of a standard (11-bit ID) CAN frame in
// order, ticked once per bit by `sync_pulse` from can_bit_timing.v
// (Step 1), and tells the rest of the controller (bit stuffing,
// CRC, arbitration, ACK - Steps 3/4/6/7) which field is currently
// active. Field lengths, in bits:
//
//   SOF=1, ARBITRATION=12 (ID[10:0]+RTR), CONTROL=6 (IDE+r0+DLC[3:0]),
//   DATA=DLC*8 (0-64), CRC=15, CRC_DELIM=1, ACK_SLOT=1, ACK_DELIM=1,
//   EOF=7, INTERMISSION=3.
//
// Scope: standard (11-bit) frames only - extended 29-bit ID frames
// are a documented stretch goal (see docs/progress_notes.md), not
// implemented here. This FSM only sequences field *boundaries* and
// bit-in-field indices; it does not itself hold the ID/data shift
// registers or drive the bus - that bit-level datapath lives in the
// top-level wrapper (Step 9), which watches `field_state` and
// `field_bit_index` to know what to load/sample each bit.
//
// Timing contract for the abort inputs (crc_error, ack_error):
// both must be valid (combinationally resolved) by the sync_pulse
// that processes field_state == CAN_FLD_CRC_DELIM / CAN_FLD_ACK_SLOT
// respectively - i.e. the caller compares computed vs. received CRC,
// or samples the ACK slot, combinationally off field_state, not one
// cycle late.
// ============================================================

module can_frame_fsm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        can_enable,

    input  wire        sync_pulse,       // once-per-bit tick (Step 1)

    // Discovered at Step 9 integration: can_bit_timing.v (Step 1)
    // pulses sync_pulse once per WIRE bit-time, including stuff
    // bits - but field_bit_index must track LOGICAL content bits
    // (field_len constants like ARBITRATION=12 count only real
    // content bits). So the top-level wrapper resolves, per role,
    // whether the wire bit-time that just completed was a stuff
    // bit (is_transmitter ? can_stuff_tx's stuff_inserted :
    // a held/reconstructed "last RX bit was a stuff bit, not a
    // stuff_error" signal - see can_controller_top.v) and presents
    // it here. On a sync_pulse with stuff_hold high, this FSM does
    // not advance at all: field_state/field_bit_index repeat for
    // one more tick, so the content generator re-presents the same
    // logical bit next tick - the same "hold if the previous tick
    // consumed a stuff bit" contract already used in tb_can_stuffing.v.
    input  wire        stuff_hold,

    // Frame start arbitration (who begins driving SOF)
    input  wire        tx_request,       // TX FIFO has a frame ready
    input  wire [3:0]  tx_dlc,           // DLC of that frame (stable for the whole attempt)
    input  wire        rx_start,         // dominant edge seen on the bus while idle

    // Mid-frame DLC capture for the receive path: the top-level's
    // Control-field shift register must present the correct DLC
    // value here by the time this FSM processes the last bit of
    // CAN_FLD_CONTROL (i.e. combinationally tracking the bits shifted
    // in so far is fine, since the DLC nibble is the last thing in
    // the Control field).
    input  wire [3:0]  rx_dlc_capture,

    // Sub-block feedback
    input  wire        arbitration_lost, // pulse during ARBITRATION (Step 6)
    input  wire        crc_error,        // valid during CRC_DELIM (Step 4 compare)
    input  wire        ack_error,        // valid during ACK_SLOT (Step 7)
    input  wire        stuff_error,      // pulse, any time (Step 3)
    input  wire        bit_error,        // pulse, any time (generic bus-monitor mismatch)

    output reg  [3:0]  field_state,      // CAN_FLD_* (can_defs.v)
    output reg  [6:0]  field_bit_index,  // 0-based index within current field
    output reg  [6:0]  field_len,        // length of current field, in bits
    output reg         is_transmitter,   // 1 = this node drives frame content

    output wire        stuff_enable,     // SOF..CRC (Step 3 window)
    output wire        crc_enable,       // SOF..end of Data (Step 4 window)
    output reg         reset_stuff,      // pulse: entering SOF
    output reg         reset_crc,        // pulse: entering SOF
    output reg         reset_arb,        // pulse: entering SOF (Step 6 arbitration latch)
    output reg         reset_ack,        // pulse: entering SOF (Step 7 held ack_error)

    output reg         frame_good,       // pulse: clean EOF completion
    output reg         frame_error,      // pulse: aborted into the error field
    output reg         go_idle           // pulse: back to IDLE, ready for next frame
);

    localparam [6:0] ERROR_FRAME_LEN = 7'd14; // 6-bit error flag + 8-bit delimiter (minimum)

    // ------------------------------------------------------------
    // Combinational field-window enables, derived from the
    // registered field_state - shared by Steps 3 and 4.
    // ------------------------------------------------------------
    assign stuff_enable =
        (field_state == `CAN_FLD_SOF)         ||
        (field_state == `CAN_FLD_ARBITRATION) ||
        (field_state == `CAN_FLD_CONTROL)     ||
        (field_state == `CAN_FLD_DATA)        ||
        (field_state == `CAN_FLD_CRC);

    assign crc_enable =
        (field_state == `CAN_FLD_SOF)         ||
        (field_state == `CAN_FLD_ARBITRATION) ||
        (field_state == `CAN_FLD_CONTROL)     ||
        (field_state == `CAN_FLD_DATA);

    // ------------------------------------------------------------
    // DLC -> Data field length (bits), capped at 8 bytes. Computed
    // combinationally so it's available in the same cycle the
    // CONTROL->DATA (or CONTROL->CRC, for DLC=0) decision is made.
    // ------------------------------------------------------------
    wire [3:0] dlc_now      = is_transmitter ? tx_dlc : rx_dlc_capture;
    wire [3:0] dlc_capped   = (dlc_now > 4'd8) ? 4'd8 : dlc_now;
    wire [6:0] data_bits_now = {dlc_capped, 3'b000}; // dlc_capped * 8

    reg [3:0] effective_dlc; // latched DLC actually used for this frame (debug/top-level use)

    always @(posedge clk) begin
        if (!rst_n || !can_enable) begin
            field_state     <= `CAN_FLD_IDLE;
            field_bit_index <= 7'd0;
            field_len       <= 7'd0;
            is_transmitter  <= 1'b0;
            reset_stuff     <= 1'b0;
            reset_crc       <= 1'b0;
            reset_arb       <= 1'b0;
            reset_ack       <= 1'b0;
            frame_good      <= 1'b0;
            frame_error     <= 1'b0;
            go_idle         <= 1'b0;
            effective_dlc   <= 4'd0;
        end
        else begin
            // Defaults for this cycle; these are all one-shot pulses.
            reset_stuff <= 1'b0;
            reset_crc   <= 1'b0;
            reset_arb   <= 1'b0;
            reset_ack   <= 1'b0;
            frame_good  <= 1'b0;
            frame_error <= 1'b0;
            go_idle     <= 1'b0;

            if (sync_pulse) begin

                // ---- Highest priority: abort into the error field ----
                if ((field_state != `CAN_FLD_IDLE) &&
                    (field_state != `CAN_FLD_ERROR) &&
                    (stuff_error || bit_error ||
                     (crc_error && (field_state == `CAN_FLD_CRC_DELIM)) ||
                     (ack_error && (field_state == `CAN_FLD_ACK_SLOT)))) begin

                    field_state     <= `CAN_FLD_ERROR;
                    field_bit_index <= 7'd0;
                    field_len       <= ERROR_FRAME_LEN;
                    frame_error     <= 1'b1;
                end
                else if (stuff_hold) begin
                    // The wire bit-time that just completed was a
                    // stuff bit, not a logical content bit - hold
                    // field_state/field_bit_index steady for this
                    // tick (no assignment needed; regs keep their
                    // value by default).
                end
                else begin
                    case (field_state)

                        `CAN_FLD_IDLE: begin
                            if (tx_request || rx_start) begin
                                field_state     <= `CAN_FLD_SOF;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd1;
                                // If both a local TX request and an
                                // external edge land on the same
                                // sync_pulse, attempt transmission -
                                // real bitwise arbitration (Step 6)
                                // resolves any actual collision during
                                // the Arbitration field, same as two
                                // independent nodes starting at once.
                                is_transmitter  <= tx_request;
                                reset_stuff     <= 1'b1;
                                reset_crc       <= 1'b1;
                                reset_arb       <= 1'b1;
                                reset_ack       <= 1'b1;
                            end
                        end

                        `CAN_FLD_SOF: begin
                            field_state     <= `CAN_FLD_ARBITRATION;
                            field_bit_index <= 7'd0;
                            field_len       <= 7'd12;
                        end

                        `CAN_FLD_ARBITRATION: begin
                            if (arbitration_lost)
                                is_transmitter <= 1'b0;

                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_CONTROL;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd6;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_CONTROL: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                effective_dlc <= dlc_now;

                                if (data_bits_now == 7'd0) begin
                                    // DLC=0: no Data field bits at all -
                                    // go straight to CRC, don't spend a
                                    // bit-time "in" an empty Data field.
                                    field_state     <= `CAN_FLD_CRC;
                                    field_bit_index <= 7'd0;
                                    field_len       <= 7'd15;
                                end
                                else begin
                                    field_state     <= `CAN_FLD_DATA;
                                    field_bit_index <= 7'd0;
                                    field_len       <= data_bits_now;
                                end
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_DATA: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_CRC;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd15;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_CRC: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_CRC_DELIM;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd1;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_CRC_DELIM: begin
                            field_state     <= `CAN_FLD_ACK_SLOT;
                            field_bit_index <= 7'd0;
                            field_len       <= 7'd1;
                        end

                        `CAN_FLD_ACK_SLOT: begin
                            field_state     <= `CAN_FLD_ACK_DELIM;
                            field_bit_index <= 7'd0;
                            field_len       <= 7'd1;
                        end

                        `CAN_FLD_ACK_DELIM: begin
                            field_state     <= `CAN_FLD_EOF;
                            field_bit_index <= 7'd0;
                            field_len       <= 7'd7;
                        end

                        `CAN_FLD_EOF: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_INTERMISSION;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd3;
                                frame_good      <= 1'b1;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_INTERMISSION: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_IDLE;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd0;
                                go_idle         <= 1'b1;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        `CAN_FLD_ERROR: begin
                            if (field_bit_index + 7'd1 >= field_len) begin
                                field_state     <= `CAN_FLD_INTERMISSION;
                                field_bit_index <= 7'd0;
                                field_len       <= 7'd3;
                            end
                            else begin
                                field_bit_index <= field_bit_index + 7'd1;
                            end
                        end

                        default: begin
                            field_state     <= `CAN_FLD_IDLE;
                            field_bit_index <= 7'd0;
                        end

                    endcase
                end
            end
        end
    end

endmodule
