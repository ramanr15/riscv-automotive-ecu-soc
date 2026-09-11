`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_destuff_rx.v
//
// WBS Section 2.3.4 - Step 3 (receive path): Bit Destuffing
//
// Mirrors can_stuff_tx.v: tracks consecutive identical bits
// actually observed on the (already-synchronized) incoming bit
// stream. When five identical bits have been seen, the next bit
// is a stuff bit - it is discarded (not handed to the frame
// decoder) and must be the opposite polarity, or a stuff error is
// raised.
//
// `bit_tick` is expected to be driven from the bit timing
// generator's sample_point (Step 1) while stuff_enable (SOF..CRC
// field, from the Step 5 frame FSM) is high.
// ============================================================

module can_destuff_rx (
    input  wire clk,
    input  wire rst_n,

    input  wire stuff_enable, // 1 during SOF..CRC field
    input  wire reset_stuff,  // pulse at SOF: prime the run counter
    input  wire bit_tick,     // one pulse per sampled CAN bit
    input  wire rx_bit,       // sampled bus level this bit-time

    output reg          valid_bit,   // 1 => rx_bit is real frame content
    output reg  [2:0]   same_count,  // debug: consecutive identical bits seen
    output reg          stuff_error  // 1-clk pulse: expected stuff bit was wrong polarity
);

    reg last_bit;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_bit   <= 1'b0;
            same_count  <= 3'd0;
            stuff_error <= 1'b0;
            last_bit    <= 1'b1; // bus idles recessive
        end
        else if (reset_stuff) begin
            same_count  <= 3'd0;
            last_bit    <= 1'b1;
            valid_bit   <= 1'b0;
            stuff_error <= 1'b0;
        end
        else if (bit_tick) begin

            stuff_error <= 1'b0; // default; asserted below for exactly 1 clk

            if (stuff_enable && (same_count == 3'd5)) begin
                // This bit must be the inserted stuff bit.
                if (rx_bit == ~last_bit) begin
                    valid_bit  <= 1'b0; // discard - not frame content
                    same_count <= 3'd1;
                    last_bit   <= rx_bit;
                end
                else begin
                    // Stuffing rule violated: 6th consecutive
                    // identical bit observed. Flag it; the error
                    // state machine (Step 8) and frame FSM (Step 5)
                    // decide how to abort/react.
                    valid_bit   <= 1'b0;
                    stuff_error <= 1'b1;
                    same_count  <= 3'd1;
                    last_bit    <= rx_bit;
                end
            end
            else if (stuff_enable) begin
                valid_bit <= 1'b1;

                if (rx_bit == last_bit)
                    same_count <= same_count + 3'd1;
                else
                    same_count <= 3'd1;

                last_bit <= rx_bit;
            end
            else begin
                // Outside the stuffed region: pass through, no
                // stuffing accounting (CRC delimiter, ACK, EOF).
                valid_bit  <= 1'b1;
                same_count <= 3'd0;
                last_bit   <= rx_bit;
            end
        end
        else begin
            valid_bit <= 1'b0; // no new bit this cycle
        end
    end

endmodule
