`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_ack.v
//
// WBS Section 2.3.8 - Step 7: ACK Handling
//
// Transmitter: during the ACK slot, drive recessive, then sample
// the bus. Dominant means some receiver acknowledged the frame;
// recessive means nobody did - raise an ACK error.
//
// Receiver: during the ACK slot, if the frame's CRC checked out,
// drive the bus dominant for that one bit - even if this node
// doesn't otherwise care about the frame's contents (every node
// that received it cleanly acks, per spec).
//
// Design note - no separate `crc_ok` input: reaching the ACK Slot
// field at all already implies this node's own CRC check passed.
// The frame FSM's (Step 5) abort-into-error-field check has top
// priority over every ordinary field transition, and it fires on
// `crc_error` at CAN_FLD_CRC_DELIM *before* CAN_FLD_ACK_SLOT is
// ever reached - so a receiver with a bad CRC never gets here in
// the first place. This block can therefore just unconditionally
// drive dominant when receiving, relying on that upstream
// invariant rather than re-deciding it - flagged here explicitly
// so Person 3's verification plan knows why there's no CRC input
// on this module.
//
// `ack_error` is a held level, not a one-shot pulse: it's sampled
// (registered) at `sample_point`, but must stay valid through the
// later `sync_pulse` that exits CAN_FLD_ACK_SLOT, per the frame
// FSM's own documented timing contract for its abort inputs. It's
// cleared by `reset_ack` (pulsed at SOF, same as reset_stuff/
// reset_crc/reset_arb).
// ============================================================

module can_ack (
    input  wire clk,
    input  wire rst_n,

    input  wire reset_ack,       // pulse at SOF (Step 5): clear the held ack_error
    input  wire ack_enable,      // 1 while field_state == CAN_FLD_ACK_SLOT (Step 5)
    input  wire sample_point,    // 1-clk pulse, PHASE1/PHASE2 boundary (Step 1)

    input  wire is_transmitter,  // this node's role for the current frame (Step 5)
    input  wire bus_bit,         // monitored bus level (RX pin) during the ACK slot

    output wire ack_drive,       // combinational: bit this node should drive during
                                  // CAN_FLD_ACK_SLOT (top-level muxes this onto can_tx
                                  // only while ack_enable is high)
    output reg  ack_error        // held: set if, while transmitting, the sampled ACK
                                  // slot was recessive (no one acknowledged)
);

    // Transmitter must stay recessive so a receiver's dominant pull
    // is visible; a receiver that got this far (see header note)
    // always acks dominant.
    assign ack_drive = is_transmitter ? `CAN_RECESSIVE : `CAN_DOMINANT;

    always @(posedge clk) begin
        if (!rst_n) begin
            ack_error <= 1'b0;
        end
        else if (reset_ack) begin
            ack_error <= 1'b0;
        end
        else if (sample_point && ack_enable && is_transmitter) begin
            ack_error <= (bus_bit == `CAN_RECESSIVE);
        end
        // else: hold - not sampling this cycle, so ack_error keeps
        // whatever value it last settled to this frame.
    end

endmodule
