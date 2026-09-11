`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_bit_timing.v
//
// WBS Section 2.3.2 - Step 1: Bit Timing Generator
//
// Divides the system clock down to the CAN bit rate and walks
// through the four segments of a CAN bit time:
//
//     SYNC_SEG -> PROP_SEG -> PHASE_SEG1 -> PHASE_SEG2 -> (repeat)
//
// sample_point pulses for one Time Quantum (TQ) at the boundary
// between PHASE_SEG1 and PHASE_SEG2 - this is when the bus level
// is latched by the rest of the controller.
//
// Resynchronization (ISO 11898-1 8.2, simplified for an
// automotive-scope student/portfolio project):
//   - hard_sync   : forces an immediate restart at SYNC_SEG.
//                   Used once, on the recessive->dominant edge
//                   that begins Start-of-Frame while this node is
//                   idle/receiving (i.e. not just after its own
//                   SYNC_SEG).
//   - resync_edge : an unexpected recessive->dominant edge seen
//                   mid-bit while this node is a receiver (not
//                   the edge that opens SOF). If the edge arrives
//                   before the sample point (during PROP/PHASE1),
//                   the remainder of that segment is shortened by
//                   up to SJW time quanta (phase-advance). If it
//                   arrives after the sample point (during
//                   PHASE2), PHASE2 is lengthened by up to SJW
//                   time quanta before the next SYNC_SEG (phase-
//                   delay). Both corrections saturate at SJW, per
//                   spec.
//
// Known simplification (documented for Person 3's verification
// plan): hard_sync/resync_edge are only actioned on a tq_pulse
// boundary, i.e. synchronization is quantized to one Time
// Quantum rather than truly asynchronous to the bus edge. A
// fully compliant implementation would restart the prescaler
// itself on the triggering edge. This is adequate for the
// project's stated bit-timing test (config the segments, toggle
// the bus, confirm sample_point lands at the hand-calculated TQ)
// and for multi-node arbitration/ACK on the same board clock,
// but should be flagged if bit-rate mismatch between real,
// independently-clocked ECUs is ever tested on hardware.
// ============================================================

module can_bit_timing (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        can_enable,     // controller enabled (CAN_CTRL.EN)

    // CAN_BTIME fields (see docs/can_register_map.md for the
    // exact bit layout delivered to Person 2 / Person 3)
    input  wire [7:0]  brp,            // prescale divide = brp + 1
    input  wire [3:0]  prop_seg,       // TQ count (0 treated as 1)
    input  wire [3:0]  phase_seg1,     // TQ count (0 treated as 1)
    input  wire [3:0]  phase_seg2,     // TQ count (0 treated as 1)
    input  wire [1:0]  sjw,            // max resync jump, in TQ (0 treated as 1)

    input  wire        hard_sync,      // pulse: restart at SYNC_SEG
    input  wire        resync_edge,    // pulse: unexpected edge seen

    output reg         tq_pulse,       // 1 clk wide, once per time quantum
    output reg         sample_point,   // 1 clk wide, PHASE1/PHASE2 boundary
    output reg         sync_pulse,     // 1 clk wide, once per bit: the SYNC_SEG/PROP_SEG
                                        // boundary (fires after SYNC_SEG's 1 TQ has
                                        // elapsed, same "trailing boundary" convention as
                                        // sample_point below - NOT a "start of frame only"
                                        // signal, it pulses once per CAN bit, every bit).
                                        // The frame FSM (Step 5) uses this as its generic
                                        // per-bit tick, and separately pulses reset_stuff/
                                        // reset_crc on the specific bit where it enters the
                                        // SOF field.
    output reg  [2:0]  seg_state       // debug/observability
);

    // ------------------------------------------------------------
    // Segment encoding
    // ------------------------------------------------------------
    localparam SEG_SYNC   = 3'd0;
    localparam SEG_PROP   = 3'd1;
    localparam SEG_PHASE1 = 3'd2;
    localparam SEG_PHASE2 = 3'd3;

    // Treat a 0 programmed length as 1 TQ so a lazily-initialized
    // register can't wedge the state machine.
    wire [3:0] prop_len   = (prop_seg   == 4'd0) ? 4'd1 : prop_seg;
    wire [3:0] phase1_len = (phase_seg1 == 4'd0) ? 4'd1 : phase_seg1;
    wire [3:0] phase2_len = (phase_seg2 == 4'd0) ? 4'd1 : phase_seg2;
    wire [1:0] sjw_len    = (sjw        == 2'd0) ? 2'd1 : sjw;

    // ------------------------------------------------------------
    // Prescaler: system clock -> one-TQ-wide tq_pulse
    // ------------------------------------------------------------
    reg [7:0] presc_cnt;

    always @(posedge clk) begin
        if (!rst_n || !can_enable) begin
            presc_cnt <= 8'd0;
            tq_pulse  <= 1'b0;
        end
        else if (presc_cnt == brp) begin
            presc_cnt <= 8'd0;
            tq_pulse  <= 1'b1;
        end
        else begin
            presc_cnt <= presc_cnt + 8'd1;
            tq_pulse  <= 1'b0;
        end
    end

    // ------------------------------------------------------------
    // Segment walker
    //
    // All TQ counters/lengths are carried at 5 bits internally
    // (one bit wider than the 4-bit register fields) so that
    // "length + SJW correction" style comparisons below can never
    // wrap around, even at the largest legal programmed values.
    // ------------------------------------------------------------
    reg [4:0] tq_in_seg;      // TQs elapsed in the current segment
    reg [4:0] extra_tq;       // outstanding PHASE2 resync stretch, in TQ

    wire [4:0] prop_len5   = {1'b0, prop_len};
    wire [4:0] phase1_len5 = {1'b0, phase1_len};
    wire [4:0] phase2_len5 = {1'b0, phase2_len};
    wire [4:0] sjw_len5    = {3'b000, sjw_len};

    // TQs remaining to the end of PHASE1, from the *current* tq_in_seg
    // (evaluated before this TQ is consumed).
    wire [4:0] phase1_remaining = phase1_len5 - tq_in_seg;

    // Registered "restart at SYNC" request so a hard_sync pulse that
    // lands between tq_pulses is not lost.
    reg pending_hard_sync;

    always @(posedge clk) begin
        if (!rst_n) pending_hard_sync <= 1'b0;
        else if (hard_sync) pending_hard_sync <= 1'b1;
        else if (tq_pulse) pending_hard_sync <= 1'b0;
    end

    always @(posedge clk) begin
        if (!rst_n || !can_enable) begin
            seg_state    <= SEG_SYNC;
            tq_in_seg    <= 5'd0;
            extra_tq     <= 5'd0;
            sample_point <= 1'b0;
            sync_pulse   <= 1'b0;
        end
        else begin
            // Defaults for this cycle; overridden below when a
            // segment boundary or sync event happens on this edge.
            sample_point <= 1'b0;
            sync_pulse   <= 1'b0;

            if (tq_pulse) begin

                if (hard_sync || pending_hard_sync) begin
                    // Hard synchronization: jump straight to the
                    // start of a fresh bit time.
                    seg_state <= SEG_SYNC;
                    tq_in_seg <= 5'd0;
                    extra_tq  <= 5'd0;
                    sync_pulse <= 1'b1;
                end
                else begin
                    case (seg_state)

                        SEG_SYNC: begin
                            sync_pulse <= 1'b1;
                            seg_state <= SEG_PROP;
                            tq_in_seg <= 5'd0;
                        end

                        SEG_PROP: begin
                            if (tq_in_seg + 5'd1 >= prop_len5) begin
                                seg_state <= SEG_PHASE1;
                                tq_in_seg <= 5'd0;
                            end
                            else begin
                                tq_in_seg <= tq_in_seg + 5'd1;
                            end
                        end

                        SEG_PHASE1: begin
                            // An edge seen while still in PROP/PHASE1
                            // means our clock is running late relative
                            // to the transmitter: phase-advance by
                            // jumping to the sample point now, bounded
                            // to at most SJW time quanta of correction.
                            if (resync_edge && !hard_sync &&
                                (phase1_remaining > sjw_len5)) begin
                                tq_in_seg <= tq_in_seg + sjw_len5;
                            end
                            else if (resync_edge && !hard_sync) begin
                                // Remaining segment is within one SJW
                                // jump: go straight to the sample point.
                                sample_point <= 1'b1;
                                seg_state    <= SEG_PHASE2;
                                tq_in_seg    <= 5'd0;
                            end
                            else if (tq_in_seg + 5'd1 >= phase1_len5) begin
                                sample_point <= 1'b1;
                                seg_state    <= SEG_PHASE2;
                                tq_in_seg    <= 5'd0;
                            end
                            else begin
                                tq_in_seg <= tq_in_seg + 5'd1;
                            end
                        end

                        SEG_PHASE2: begin
                            // A late edge here means our clock is
                            // running early: phase-delay by stretching
                            // PHASE2 up to SJW extra time quanta,
                            // latched once per bit time.
                            if (resync_edge && !hard_sync &&
                                (extra_tq < sjw_len5) &&
                                (tq_in_seg + 5'd1 < phase2_len5 + extra_tq)) begin
                                extra_tq  <= extra_tq + 5'd1;
                                tq_in_seg <= tq_in_seg + 5'd1;
                            end
                            else if (tq_in_seg + 5'd1 >= phase2_len5 + extra_tq) begin
                                seg_state <= SEG_SYNC;
                                tq_in_seg <= 5'd0;
                                extra_tq  <= 5'd0;
                            end
                            else begin
                                tq_in_seg <= tq_in_seg + 5'd1;
                            end
                        end

                        default: begin
                            seg_state <= SEG_SYNC;
                            tq_in_seg <= 5'd0;
                        end

                    endcase
                end
            end
        end
    end

endmodule
