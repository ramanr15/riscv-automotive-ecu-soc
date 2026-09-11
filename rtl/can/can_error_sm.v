`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_error_sm.v
//
// WBS Section 2.3.9 - Step 8: Error State Machine
// ("What Makes This 'Automotive-Grade'")
//
// CAN's fault confinement: two saturating error counters (TEC,
// REC) drive three states - Error-Active, Error-Passive, Bus-Off -
// per ISO 11898-1.
//
// Counting rule implemented (documented explicitly, per the WBS's
// own instruction, since this is "the part a verification engineer
// will scrutinize most"): this project uses the general rule
// uniformly for both roles - a detected error (`frame_error`, the
// frame FSM's Step 5 abort-into-error-field pulse) increases the
// relevant counter by 8 (TEC if this node was transmitting that
// frame, REC if receiving); a cleanly completed frame
// (`frame_good`) decreases the relevant counter by 1, floored at
// 0. The full CAN spec has additional asymmetric edge-case rules
// (e.g. a receiver that is first to detect a stuff/form/bit error
// gets a different increment than one that only sees a later
// active error flag; a transmitter with an already-set passive
// error flag has its own rule) - NOT implemented here, called out
// as a documented simplification consistent with the rest of this
// project's scope (see docs/progress_notes.md).
//
// Counters are tracked internally at 10 bits (0-511) - wider than
// the 8-bit "TEC exceeds 255" Bus-Off test requires headroom for -
// and the externally visible `tec`/`rec` are saturated/clamped to
// 8 bits (0-255) for register-file display (Step 9), per the WBS's
// own "8-bit (saturating, but tracked beyond 8 bits internally)"
// instruction.
//
// State register timing: `err_state`/`err_state_changed` are
// computed from the error counters' value as of the *previous*
// clock (i.e. one system clock behind a counter update that
// crosses a threshold) - a deliberate, uniformly-applied
// simplification, harmless at CAN bit-time granularity (a system
// clock is a small fraction of a bit time) and documented here so
// it isn't mistaken for a bug. `tec`/`rec` themselves are plain
// combinational reads of the internal counters and carry no such
// lag.
//
// Bus-Off recovery: while in Bus-Off, this block silently counts
// bus activity via `sample_point`/`bus_bit` (Step 1) - 128
// SEPARATE occurrences of 11 consecutive recessive bits, per spec.
// A dominant bit interrupts the current run but does not undo
// already-completed occurrences. On the 128th completed occurrence
// both counters are cleared, which naturally returns `err_state`
// to Error-Active one clock later via the normal state-register
// logic above.
// ============================================================

module can_error_sm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        can_enable,

    input  wire        frame_good,        // pulse: frame completed cleanly (Step 5)
    input  wire        frame_error,       // pulse: frame aborted into the error field (Step 5)
    input  wire        is_transmitter,    // this frame's role, valid at frame_good/frame_error (Step 5)

    input  wire        sample_point,      // Step 1 - used only for Bus-Off recovery counting
    input  wire        bus_bit,           // monitored bus level (RX pin)

    output wire [7:0]  tec,               // Transmit Error Counter, saturated 0-255
    output wire [7:0]  rec,               // Receive Error Counter, saturated 0-255
    output reg  [1:0]  err_state,         // CAN_ERR_ACTIVE/PASSIVE/BUSOFF (can_defs.v)
    output reg         err_state_changed  // pulse: err_state just changed - raise CAN Error IRQ (Step 9)
);

    localparam [9:0] SAT_MAX = 10'd511; // internal counter ceiling, defensive only

    reg [9:0] tec_cnt, rec_cnt;      // wider-than-display internal counters
    reg [3:0] recessive_run;         // consecutive recessive bits seen this run (0-11)
    reg [7:0] recovery_run_count;    // completed 11-bit recessive runs seen (0-128)

    assign tec = (tec_cnt > 10'd255) ? 8'd255 : tec_cnt[7:0];
    assign rec = (rec_cnt > 10'd255) ? 8'd255 : rec_cnt[7:0];

    wire [1:0] next_state =
        (tec_cnt > 10'd255)                            ? `CAN_ERR_BUSOFF  :
        ((tec_cnt >= 10'd128) || (rec_cnt >= 10'd128))  ? `CAN_ERR_PASSIVE :
                                                           `CAN_ERR_ACTIVE;

    always @(posedge clk) begin
        if (!rst_n || !can_enable) begin
            tec_cnt            <= 10'd0;
            rec_cnt            <= 10'd0;
            recessive_run      <= 4'd0;
            recovery_run_count <= 8'd0;
            err_state          <= `CAN_ERR_ACTIVE;
            err_state_changed  <= 1'b0;
        end
        else begin

            if (err_state == `CAN_ERR_BUSOFF) begin
                // Ordinary frame accounting is frozen; only the
                // recovery sequence below can move the counters.
                if (sample_point) begin
                    if (bus_bit == `CAN_RECESSIVE) begin
                        if (recessive_run + 4'd1 >= 4'd11) begin
                            recessive_run <= 4'd0;
                            if (recovery_run_count + 8'd1 >= 8'd128) begin
                                // 128th completed occurrence: recover.
                                tec_cnt            <= 10'd0;
                                rec_cnt            <= 10'd0;
                                recovery_run_count <= 8'd0;
                            end
                            else begin
                                recovery_run_count <= recovery_run_count + 8'd1;
                            end
                        end
                        else begin
                            recessive_run <= recessive_run + 4'd1;
                        end
                    end
                    else begin
                        // Dominant bit: interrupts the current run
                        // only - already-completed occurrences stand.
                        recessive_run <= 4'd0;
                    end
                end
            end
            else begin
                // Not in Bus-Off: recovery bookkeeping stays clear,
                // ready for a future Bus-Off event.
                recessive_run      <= 4'd0;
                recovery_run_count <= 8'd0;

                if (frame_error) begin
                    if (is_transmitter)
                        tec_cnt <= (tec_cnt + 10'd8 > SAT_MAX) ? SAT_MAX : tec_cnt + 10'd8;
                    else
                        rec_cnt <= (rec_cnt + 10'd8 > SAT_MAX) ? SAT_MAX : rec_cnt + 10'd8;
                end
                else if (frame_good) begin
                    if (is_transmitter)
                        tec_cnt <= (tec_cnt == 10'd0) ? 10'd0 : tec_cnt - 10'd1;
                    else
                        rec_cnt <= (rec_cnt == 10'd0) ? 10'd0 : rec_cnt - 10'd1;
                end
            end

            err_state         <= next_state;
            err_state_changed <= (next_state != err_state);
        end
    end

endmodule
