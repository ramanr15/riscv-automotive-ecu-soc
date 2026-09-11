`timescale 1ns / 1ps

// ============================================================
// can_crc15.v
//
// WBS Section 2.3.5 - Step 4: CRC-15 Generator and Checker
//
// Standard CAN CRC-15, polynomial
//   x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
// implemented as the well-known bit-serial LFSR (constant 15'h4599
// is the polynomial with the implicit leading x^15 term dropped,
// per the Bosch CAN 2.0 reference algorithm):
//
//   crc_nxt = data_bit ^ crc_reg[14];
//   crc_reg = {crc_reg[13:0], 1'b0};
//   if (crc_nxt) crc_reg = crc_reg ^ 15'h4599;
//
// This single module is shared by both the TX path (computing the
// CRC to append after the Data field) and the RX path (recomputing
// the CRC over the received bits to compare against the received
// CRC field) - instantiate one per direction.
//
// IMPORTANT: `shift_en` must pulse once per *logical* content bit
// - i.e. Start-of-Frame through the last bit of the Data field -
// and must NOT pulse for bits that bit stuffing (Step 3) inserted
// or removed. On TX that means gating with `bit_tick && !stuff_inserted`
// from can_stuff_tx; on RX it means gating with `bit_tick && valid_bit`
// from can_destuff_rx. This module knows nothing about stuffing
// itself, which keeps it simple to reuse.
//
// Integration note (found at Step 9b, real top-level wiring): on the
// TX instance specifically, `reset_crc` and the SOF bit's own
// `shift_en` pulse land on the exact same clock edge (both are
// one-cycle-registered echoes of the same originating sync_pulse,
// same root cause as the analogous can_stuff_tx.v fix). The original
// `if (!rst_n || reset_crc) ... else if (shift_en)` priority let a
// coincident reset_crc silently swallow that edge's shift, dropping
// the SOF bit from the CRC accumulation entirely - on the RX
// instance this never showed up, since its shift_en (gated by
// valid_bit, which pulses several cycles later in the bit time) never
// coincides with reset_crc, so the two instances would silently
// accumulate a different number of bits and never match. Fixed by
// computing the "effective" pre-shift crc_reg baseline as zero
// whenever reset_crc is asserted this same edge, and using that
// baseline for the shift - this is exactly the standard "reset value
// then shift the first bit in" CRC behavior, and is unchanged for
// reset_crc pulsing with no coincident shift_en (crc_reg simply
// resets, same as before).
// ============================================================

module can_crc15 (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        reset_crc,  // pulse at SOF: crc_reg <= 0
    input  wire        shift_en,   // pulse per logical content bit
    input  wire        data_bit,

    output reg  [14:0] crc_reg     // live CRC register value
);

    localparam [14:0] CAN_CRC15_POLY = 15'h4599;

    // Effective pre-shift baseline for this edge: zero if reset_crc
    // is asserted right now (even though the real crc_reg register
    // hasn't been written yet this edge), so a shift_en that
    // coincides with reset_crc (the real SOF case) still correctly
    // shifts its bit into a freshly-cleared register on this same
    // edge, instead of that shift being dropped.
    wire [14:0] base_crc_reg = reset_crc ? 15'd0 : crc_reg;
    wire        crc_nxt      = data_bit ^ base_crc_reg[14];

    always @(posedge clk) begin
        if (!rst_n) begin
            crc_reg <= 15'd0;
        end
        else if (shift_en) begin
            if (crc_nxt)
                crc_reg <= {base_crc_reg[13:0], 1'b0} ^ CAN_CRC15_POLY;
            else
                crc_reg <= {base_crc_reg[13:0], 1'b0};
        end
        else if (reset_crc) begin
            crc_reg <= 15'd0;
        end
    end

endmodule
