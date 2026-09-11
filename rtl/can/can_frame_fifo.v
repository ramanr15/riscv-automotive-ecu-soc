`timescale 1ns / 1ps

// ============================================================
// can_frame_fifo.v
//
// WBS Section 2.3.3 - Step 2: TX and RX FIFOs
//
// Buffers whole CAN frames (not bits), depth 8 by default, so the
// CPU does not have to service the bus in real time. One instance
// of this module is used for TX (Section 2.3.9 CAN_TX_* registers
// push into it) and a second instance for RX (CAN_RX_* registers
// pop from it).
//
// Frame fields stored per entry:
//   id[28:0]   - 11-bit ID in id[10:0] for standard frames, or the
//                full 29-bit extended ID when ide=1
//   ide        - 0 = standard (11-bit), 1 = extended (29-bit)
//   rtr        - Remote Transmission Request bit
//   dlc[3:0]   - Data Length Code, 0-8 bytes
//   data[63:0] - up to 8 data bytes, data[7:0] is byte 0, etc.
//
// Pointer scheme: each pointer is one bit wider than needed to
// index DEPTH entries. Equal pointers with equal extra bit ->
// empty. Equal pointers with differing extra bit -> full. This is
// the same standard technique Person 2's reusable byte-FIFO
// template uses (Section 3.5), kept consistent across the SoC even
// though this FIFO is frame-wide rather than byte-wide.
// ============================================================

module can_frame_fifo #(
    parameter DEPTH = 8   // must be a power of two
) (
    input  wire        clk,
    input  wire        rst_n,

    // Write (push) side
    input  wire        push,
    input  wire [28:0] id_in,
    input  wire        ide_in,
    input  wire        rtr_in,
    input  wire [3:0]  dlc_in,
    input  wire [63:0] data_in,

    // Read (pop) side
    input  wire        pop,
    output reg  [28:0] id_out,
    output reg         ide_out,
    output reg         rtr_out,
    output reg  [3:0]  dlc_out,
    output reg  [63:0] data_out,

    output wire        full,
    output wire        empty,

    // One-clock pulses for interrupt generation (Section 2.3.3):
    // RX FIFO: empty -> non-empty (frame arrived)
    // TX FIFO: full -> non-full (space became available)
    output reg          empty_to_nonempty,
    output reg          full_to_nonfull
);

    localparam PTR_W = $clog2(DEPTH);

    // Entry width: id(29) + ide(1) + rtr(1) + dlc(4) + data(64) = 99
    localparam ENTRY_W = 29 + 1 + 1 + 4 + 64;

    reg [ENTRY_W-1:0] mem [0:DEPTH-1];

    reg [PTR_W:0] wr_ptr; // 1 extra bit for full/empty disambiguation
    reg [PTR_W:0] rd_ptr;

    wire [PTR_W-1:0] wr_idx = wr_ptr[PTR_W-1:0];
    wire [PTR_W-1:0] rd_idx = rd_ptr[PTR_W-1:0];

    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_idx == rd_idx) && (wr_ptr[PTR_W] != rd_ptr[PTR_W]);

    wire do_push = push && !full;
    wire do_pop  = pop  && !empty;

    // ------------------------------------------------------------
    // Write side
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {(PTR_W+1){1'b0}};
        end
        else if (do_push) begin
            mem[wr_idx] <= {data_in, dlc_in, rtr_in, ide_in, id_in};
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // ------------------------------------------------------------
    // Read side (synchronous: id/ide/rtr/dlc/data_out register the
    // entry at the head of the FIFO the cycle after `pop`, matching
    // the register map's read-then-strobe convention in Section
    // 2.3.9 / 2.4.2: CAN_RX_CTRL read is the pop strobe, and
    // CAN_RX_ID/CAN_RX_DATA0/1 are read *before* it, i.e. they must
    // already reflect the head-of-FIFO entry combinationally / on
    // the cycle it becomes the head - so the head entry is exposed
    // continuously here, and `pop` simply advances to the next one.)
    // ------------------------------------------------------------
    always @(*) begin
        {data_out, dlc_out, rtr_out, ide_out, id_out} = mem[rd_idx];
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {(PTR_W+1){1'b0}};
        end
        else if (do_pop) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // ------------------------------------------------------------
    // Edge-detected interrupt pulses
    // ------------------------------------------------------------
    reg empty_d, full_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            empty_d <= 1'b1;
            full_d  <= 1'b0;
            empty_to_nonempty <= 1'b0;
            full_to_nonfull   <= 1'b0;
        end
        else begin
            empty_d <= empty;
            full_d  <= full;

            empty_to_nonempty <= empty_d && !empty;
            full_to_nonfull   <= full_d  && !full;
        end
    end

endmodule
