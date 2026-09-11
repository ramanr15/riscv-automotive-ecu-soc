`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_arbitration.v
//
// WBS Section 2.3.7 - Step 6: Arbitration Logic
//
// CAN's non-destructive bitwise arbitration: every node competing
// for the bus keeps driving its own wire bits during the
// Arbitration field, while simultaneously monitoring the actual
// bus level. Dominant (0) always wins over recessive (1). If a
// node drives recessive but reads back dominant, some other node
// is driving a lower (higher-priority) ID, and this node has lost
// - it must stop driving and switch to receive-only for the rest
// of the frame (the frame FSM, Step 5, does that: it clears
// is_transmitter on `arbitration_lost`).
//
// Sampling point: comparison happens at `sample_point` (the
// PHASE1/PHASE2 boundary from can_bit_timing.v, Step 1) rather
// than at the earlier `sync_pulse` bit boundary - PROP_SEG exists
// specifically so a driven bit has time to propagate to every
// node on the bus and be read back correctly, so the bus level is
// only trustworthy once PROP_SEG (and part of PHASE1) has
// elapsed, same convention already used for stuff/CRC bit errors.
//
// `my_bit` is expected to be the actual driven wire bit
// (post-stuffing, i.e. can_stuff_tx's `tx_bit`, Step 3) rather
// than the raw content bit - bit stuffing's window already
// includes the Arbitration field, and a still-tied competitor
// independently inserts the identical stuff bit at the identical
// wire position (its bit history up to that point is identical to
// this node's, since every bit compared equal so far), so
// comparing on the stuffed stream is both correct and simpler than
// trying to exempt stuff bits from arbitration.
//
// Scoped to the Arbitration field only (`arb_enable`), matching
// the frame FSM's own contract: it only reacts to
// `arbitration_lost` while `field_state == CAN_FLD_ARBITRATION`.
// SOF itself is always dominant for every contending node, so no
// real arbitration decision happens there.
// ============================================================

module can_arbitration (
    input  wire clk,
    input  wire rst_n,

    input  wire reset_arb,        // pulse at SOF (Step 5): clear the "already lost" latch
    input  wire arb_enable,       // 1 while field_state == CAN_FLD_ARBITRATION (Step 5)
    input  wire sample_point,     // 1-clk pulse, PHASE1/PHASE2 boundary (Step 1)

    input  wire my_bit,           // this node's actual driven wire bit this bit-time (Step 3 tx_bit)
    input  wire bus_bit,          // monitored bus level this bit-time (RX pin)

    output reg  arbitration_lost, // pulse: my_bit recessive, bus_bit dominant -> lost this bit
    output reg  lost_latched      // debug/observability: stays set for the rest of the frame
);

    always @(posedge clk) begin
        if (!rst_n) begin
            arbitration_lost <= 1'b0;
            lost_latched     <= 1'b0;
        end
        else if (reset_arb) begin
            // New frame attempt: still in the running until proven
            // otherwise.
            arbitration_lost <= 1'b0;
            lost_latched     <= 1'b0;
        end
        else begin
            arbitration_lost <= 1'b0; // default: one-shot pulse

            if (sample_point && arb_enable && !lost_latched) begin
                if ((my_bit == `CAN_RECESSIVE) && (bus_bit == `CAN_DOMINANT)) begin
                    arbitration_lost <= 1'b1;
                    lost_latched     <= 1'b1;
                end
            end
        end
    end

endmodule
