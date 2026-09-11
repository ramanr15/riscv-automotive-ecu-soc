`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_stuff_tx.v
//
// WBS Section 2.3.4 - Step 3 (transmit path): Bit Stuffing
//
// CAN inserts an opposite-polarity stuff bit after five
// consecutive identical bits on the wire, so the receiver's clock
// can resynchronize. This block sits between the frame content
// shifter (driven by the frame field state machine, Step 5) and
// the physical can_tx pin.
//
// Timing model: the caller pulses `bit_tick` once per CAN bit
// time (aligned by the bit timing generator / frame FSM to the
// start of each bit). On a `bit_tick` while `stuff_enable` is
// high (SOF through the end of the CRC field, per Section 2.3.4 -
// the frame FSM drives stuff_enable, not this module):
//
//   - if 5 identical bits have already gone out on the wire,
//     this bit-time forces the OPPOSITE bit onto tx_bit and does
//     NOT consume raw_bit (raw_bit stays queued for the next
//     bit-time) - `stuff_inserted` is asserted so the caller
//     knows not to advance its shifter.
//   - otherwise raw_bit is passed straight through and consumed.
//
// `reset_stuff` is pulsed by the frame FSM at Start-of-Frame: the
// dominant SOF bit is bit #1 of a fresh run, so same_count is
// primed to 1 rather than 0.
//
// Integration note (found at Step 9b, real top-level wiring, not
// caught by this module's own Step 3 standalone testbench): when
// this module is driven by can_frame_fsm.v/can_bit_timing.v as
// intended, `reset_stuff` and the SOF bit's OWN `bit_tick` land on
// the exact same clock edge - both are one-cycle-registered echoes
// of the identical sync_pulse that makes the frame FSM enter SOF,
// with no extra pipeline stage between them. tb_can_stuffing.v's
// standalone stimulus (reset_stuff pulsed on one edge, bit_tick on a
// later, separate edge) never exercised that coincidence, so the
// original `else if` priority chain below - which let a coincident
// reset_stuff silently swallow that same edge's bit_tick, dropping
// the SOF content bit from tx_bit entirely - passed unnoticed until
// full-system simulation. Fixed by computing the run-counter/last-
// bit baseline as "freshly reset, if reset_stuff is asserted this
// same edge" and using THAT baseline inside the bit_tick branch,
// which is now checked first. When reset_stuff arrives with no
// coincident bit_tick (tb_can_stuffing.v's own sequential usage,
// still fully valid), the trailing `else if (reset_stuff)` branch
// commits the reset exactly as before, so that testbench's behavior
// and results are unchanged.
// ============================================================

module can_stuff_tx (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        stuff_enable, // 1 during SOF..CRC field (Step 5)
    input  wire        reset_stuff,  // pulse at SOF: prime the run counter
    input  wire        bit_tick,     // one pulse per CAN bit time
    input  wire        raw_bit,      // next content bit (SOF, ID, ..., CRC)

    output reg          tx_bit,          // actual bit driven on can_tx
    output reg          stuff_inserted,  // 1 => raw_bit was NOT consumed
    output reg  [2:0]   same_count,      // debug: consecutive identical bits sent

    // Combinational (same-edge) echo of "this tick will force a stuff
    // bit and NOT consume raw_bit" - see Step 9b integration note
    // below on `stuff_now` vs. `stuff_inserted` for why a caller that
    // needs to know THIS tick's stuffing outcome (not last tick's)
    // must use this port, not the registered `stuff_inserted` above.
    output wire          stuff_now
);

    reg last_bit;

    // "Effective" run-counter baseline for this edge: if reset_stuff
    // is asserted right now, use the fresh "0 prior bits sent"
    // baseline even though the real registers haven't been written
    // yet this edge - this is what lets a bit_tick that coincides
    // with reset_stuff (the real SOF case) still process correctly
    // on the very same edge instead of being swallowed by it.
    wire [2:0] base_same_count = reset_stuff ? 3'd0 : same_count;
    wire       base_last_bit   = reset_stuff ? 1'b1 : last_bit;

    // Same-edge, purely combinational version of the "force a stuff
    // bit this tick" decision below (mirrors the `if` condition of
    // the always block exactly). `stuff_inserted` (the registered
    // output above) reports this same decision too, but one clock
    // late - fine for a caller like can_frame_fsm's `stuff_hold`,
    // which intentionally wants "was the *previous* tick's bit a
    // stuff bit" to decide whether to hold field_bit_index steady
    // *this* tick. A caller that instead needs to gate something
    // else off raw_bit *this same tick* (Step 9b integration note:
    // the top-level CRC shift-enable, which reads raw_bit
    // combinationally in lockstep with this module's own bit_tick)
    // must NOT use the registered `stuff_inserted` for that - doing
    // so gates this tick's data with *last* tick's stuffing outcome,
    // silently corrupting the CRC accumulation on every frame that
    // contains any stuffing at all. `stuff_now` is the fix: it's
    // valid the instant bit_tick fires, same edge as raw_bit.
    assign stuff_now = bit_tick && stuff_enable && (base_same_count == 3'd5);

    always @(posedge clk) begin
        if (!rst_n) begin
            tx_bit         <= `CAN_RECESSIVE;
            stuff_inserted <= 1'b0;
            same_count     <= 3'd0;
            last_bit       <= 1'b1; // bus idles recessive
        end
        else if (bit_tick) begin

            if (stuff_enable && (base_same_count == 3'd5)) begin
                // Force the stuff bit: opposite of the last five.
                tx_bit         <= ~base_last_bit;
                stuff_inserted <= 1'b1;
                same_count     <= 3'd1;
                last_bit       <= ~base_last_bit;
                // raw_bit intentionally not sampled/consumed here.
            end
            else begin
                tx_bit         <= raw_bit;
                stuff_inserted <= 1'b0;

                if (stuff_enable) begin
                    if (raw_bit == base_last_bit)
                        same_count <= base_same_count + 3'd1;
                    else
                        same_count <= 3'd1;
                end
                else begin
                    // Outside the stuffed region (CRC delimiter
                    // onward): no stuffing accounting.
                    same_count <= 3'd0;
                end

                last_bit <= raw_bit;
            end
        end
        else if (reset_stuff) begin
            // reset_stuff with no coincident bit_tick this edge
            // (e.g. a standalone caller that pulses reset_stuff and
            // only later, on a separate edge, starts ticking, same
            // as tb_can_stuffing.v): commit the reset baseline into
            // the real registers now, so the next bit_tick sees it.
            same_count     <= 3'd0;
            last_bit       <= 1'b1;
            stuff_inserted <= 1'b0;
        end
    end

endmodule
