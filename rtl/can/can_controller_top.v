`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_controller_top.v
//
// WBS Section 2.3.10/2.3.11 - Step 9b: top-level datapath wrapper.
//
// Instantiates every sub-block (Steps 1-9a) and wires them into a
// complete CAN controller: bit timing, TX/RX frame FIFOs, bit
// stuffing/destuffing, TX/RX CRC-15, the frame field state machine,
// arbitration, ACK, the error state machine, and the register file.
// Also implements the pieces that don't belong in any single Step
// module: the TX content-bit generator, the RX content-bit capture
// shift registers, the `stuff_hold` reconstruction for the RX role
// (see the header comment on `stuff_hold` in can_frame_fsm.v), the
// LOOPBACK mux (Section 2.5 self-test), and the interrupt
// pending-set pulse generation.
//
// ------------------------------------------------------------
// Design decisions made at this integration layer (not owned by
// any single sub-module, flagged here for Person 3's verification
// plan):
//
// 1. hard_sync / resync_edge (can_bit_timing.v inputs) are tied to
//    0 permanently. Rationale: this project's bit-timing generator
//    already documents (see can_bit_timing.v header) that
//    synchronization is TQ-quantized rather than truly
//    asynchronous, "adequate for... same-clock-domain bench
//    testing." Section 2.5's self-test is exactly that: a single
//    node, one system clock, no independent bus-level oscillator
//    to resynchronize against. The bit-timing generator free-runs
//    continuously once CAN_CTRL.EN is set; the frame FSM starts a
//    new frame on the next `sync_pulse` after `tx_request`/
//    `rx_start` goes high, which is always a real bit-time
//    boundary, so every field transition still lands on a
//    correctly-shaped, correctly-timed bit regardless of whether
//    hard_sync ever fires. Real multi-node, independently-clocked
//    hardware bring-up would need genuine edge-triggered
//    hard_sync/resync_edge wired from a bus edge detector - flagged
//    as a known limitation, not implemented here.
//
// 2. `bit_error` (can_frame_fsm.v's generic bus-monitor-mismatch
//    abort input) is tied to 0. No dedicated bit-monitor comparator
//    was one of the nine build steps in Section 2.3.11 (bit timing,
//    FIFOs, stuffing, CRC, frame FSM, arbitration, ACK, error SM,
//    register file) - arbitration/ACK/CRC/stuffing already each
//    have their own dedicated, spec-accurate abort trigger
//    (arbitration_lost, ack_error, crc_error, stuff_error). Adding
//    a separate whole-frame transmitted-vs-observed bit comparator
//    was out of the assigned scope; tying it off keeps the abort
//    trigger set exactly what was specified and tested.
//
// 3. LOOPBACK auto-acknowledge: in CAN_CTRL.LOOPBACK mode, the ACK
//    slot is forced dominant regardless of role, instead of using
//    can_ack's own `ack_drive` (which would have this lone node
//    stay recessive as transmitter, correctly producing an
//    unacknowledged-frame ack_error every time, since there is no
//    second node to acknowledge it). This is the same behavior real
//    CAN transceivers' internal loopback/self-test modes commonly
//    provide, and it's what makes Section 2.5's self-test
//    meaningful: a frame can complete cleanly (frame_good) and be
//    read back via CAN_RX_* end to end. can_ack.v itself is
//    unmodified; only the wire-level mux at this top level
//    substitutes the forced-dominant bit during LOOPBACK.
//    ack_error still correctly fires in non-loopback mode if a
//    transmitted frame genuinely goes unacknowledged.
//
// 4. RX FIFO push condition: `frame_good && (!is_transmitter ||
//    can_loopback)`. Real CAN nodes don't normally re-receive their
//    own transmitted frames into their own RX mailbox - but
//    LOOPBACK mode specifically means "also deliver my own TX
//    frames to my own RX side," which is what lets Section 2.5's
//    self-test verify a full TX-push -> wire encode/decode -> RX-pop
//    round trip via register reads alone.
//
// 5. TX FIFO pop condition: `frame_good && is_transmitter` (a
//    frame is retired from the TX FIFO only once it has completed
//    cleanly as this node's own transmission). Losing arbitration
//    or aborting into the error field leaves the frame queued, so
//    it is automatically retried the next time this same head
//    entry is presented - simple always-retry behavior, no
//    back-off/retry-limit counting (documented simplification
//    consistent with the rest of this project's scope).
// ------------------------------------------------------------
// ============================================================

module can_controller_top (
    input  wire        clk,
    input  wire        rst_n,

    // ---- SoC bus (Section 1.4.1) ----
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output wire [31:0] rdata,
    input  wire        we,
    input  wire        re,
    input  wire        sel,
    output wire        ready,

    // ---- Physical CAN bus pins ----
    output wire        can_tx,
    input  wire        can_rx,

    // ---- Interrupts (Section 1.4.3: RX=IRQ0, TX=IRQ1, Err=IRQ2) ----
    output wire        irq_rx,
    output wire        irq_tx,
    output wire        irq_err
);

    // ------------------------------------------------------------
    // Register file outputs (config side)
    // ------------------------------------------------------------
    wire        can_en, can_loopback, soft_reset;
    wire [7:0]  btime_brp;
    wire [3:0]  btime_prop_seg, btime_phase_seg1, btime_phase_seg2;
    wire [1:0]  btime_sjw;

    // ------------------------------------------------------------
    // Bit timing generator
    // ------------------------------------------------------------
    wire tq_pulse, sample_point, sync_pulse;
    wire [2:0] seg_state;

    can_bit_timing u_bit_timing (
        .clk(clk), .rst_n(rst_n),
        .can_enable(can_en),
        .brp(btime_brp), .prop_seg(btime_prop_seg),
        .phase_seg1(btime_phase_seg1), .phase_seg2(btime_phase_seg2),
        .sjw(btime_sjw),
        .hard_sync(1'b0), .resync_edge(1'b0),   // see design note 1
        .tq_pulse(tq_pulse), .sample_point(sample_point),
        .sync_pulse(sync_pulse), .seg_state(seg_state)
    );

    // ------------------------------------------------------------
    // Bus level / loopback mux
    // ------------------------------------------------------------
    wire tx_bit_final;                 // can_stuff_tx's driven wire bit
    assign can_tx  = tx_bit_final;
    wire bus_bit = can_loopback ? tx_bit_final : can_rx;

    // Dominant-edge detector on the perceived bus, for rx_start.
    reg bus_bit_d;
    always @(posedge clk) begin
        if (!rst_n) bus_bit_d <= `CAN_RECESSIVE;
        else        bus_bit_d <= bus_bit;
    end
    wire bus_dominant_edge = (bus_bit_d == `CAN_RECESSIVE) && (bus_bit == `CAN_DOMINANT);

    // ------------------------------------------------------------
    // Frame field state machine
    // ------------------------------------------------------------
    wire [3:0] field_state;
    wire [6:0] field_bit_index, field_len;
    wire is_transmitter;
    wire stuff_enable, crc_enable;
    wire reset_stuff, reset_crc, reset_arb, reset_ack;
    wire frame_good, frame_error, go_idle;

    // ------------------------------------------------------------
    // Forward declarations: these are each first referenced below
    // (in tx_request, the u_frame_fsm port list, and the raw_bit_tx/
    // RX-capture always blocks) before the point later in the file
    // where they'd naturally be declared next to the block that
    // produces them. Declared here up front instead, so every use
    // sees the real (multi-bit) net/reg from the start - xvlog
    // otherwise implicitly declares a 1-bit placeholder at first use
    // and then warns "already implicitly declared" when it reaches
    // the real declaration later (harmless in this Vivado version,
    // since it correctly widens the net either way, but not worth
    // relying on across simulators, so cleaned up here instead of
    // left as a standing warning on every compile).
    // ------------------------------------------------------------
    wire [28:0] tx_fifo_id_out;
    wire        tx_fifo_ide_out, tx_fifo_rtr_out;
    wire [3:0]  tx_fifo_dlc_out;
    wire [63:0] tx_fifo_data_out;
    wire        tx_fifo_full, tx_fifo_empty;
    wire        tx_fifo_empty_to_nonempty, tx_fifo_full_to_nonfull;
    wire [14:0] tx_crc_reg;
    reg  [10:0] rx_id_shift;
    reg         rx_rtr_shift;
    reg         rx_ide_shift;
    reg  [3:0]  rx_dlc_shift;
    reg  [63:0] rx_data_shift;
    reg  [14:0] rx_crc_shift;
    // field_state_wire/field_bit_index_wire: one-bit-period-delayed
    // copy of field_state/field_bit_index, used only by the RX
    // capture logic further down - see the bug #3 writeup on their
    // driving always block (right after u_stuff_tx) for why. Forward-
    // declared here for the same xvlog-implicit-declaration reason as
    // the rest of this block (they're referenced by data_abs_bit_wire
    // above their natural declaration point).
    reg  [3:0]  field_state_wire;
    reg  [6:0]  field_bit_index_wire;

    wire tx_request = can_en && !tx_fifo_empty;
    wire rx_start   = bus_dominant_edge && (field_state == `CAN_FLD_IDLE);

    wire arbitration_lost;
    wire crc_error;
    wire ack_error;
    wire stuff_error_rx;
    wire stuff_hold;

    // Only the RX destuffer can report a genuine stuffing-rule
    // violation (the TX stuffer only ever forces valid stuff bits,
    // it cannot itself detect a violation) - so the frame FSM's
    // single stuff_error input is fed from the RX destuffer only.
    // In LOOPBACK mode that RX destuffer is watching our own
    // outgoing (correctly stuffed) stream, so it will not
    // spuriously fire; on a genuine external bus it watches
    // whatever is actually arriving.
    wire frame_fsm_stuff_error = stuff_error_rx;

    can_frame_fsm u_frame_fsm (
        .clk(clk), .rst_n(rst_n), .can_enable(can_en),
        .sync_pulse(sync_pulse), .stuff_hold(stuff_hold),
        .tx_request(tx_request), .tx_dlc(tx_fifo_dlc_out), .rx_start(rx_start),
        .rx_dlc_capture(rx_dlc_shift),
        .arbitration_lost(arbitration_lost),
        .crc_error(crc_error), .ack_error(ack_error),
        .stuff_error(frame_fsm_stuff_error), .bit_error(1'b0),   // see design note 2
        .field_state(field_state), .field_bit_index(field_bit_index),
        .field_len(field_len), .is_transmitter(is_transmitter),
        .stuff_enable(stuff_enable), .crc_enable(crc_enable),
        .reset_stuff(reset_stuff), .reset_crc(reset_crc),
        .reset_arb(reset_arb), .reset_ack(reset_ack),
        .frame_good(frame_good), .frame_error(frame_error), .go_idle(go_idle)
    );

    wire arb_enable = (field_state == `CAN_FLD_ARBITRATION);
    wire ack_enable = (field_state == `CAN_FLD_ACK_SLOT);

    // ------------------------------------------------------------
    // TX side: FIFO head -> content-bit generator -> stuffer -> pin
    // (tx_fifo_id_out/ide_out/rtr_out/dlc_out/data_out/full/empty/
    // empty_to_nonempty/full_to_nonfull are declared earlier, in the
    // forward-declarations block above.)
    // ------------------------------------------------------------
    wire        tx_push;
    wire [28:0] tx_push_id;
    wire        tx_push_ide, tx_push_rtr;
    wire [3:0]  tx_push_dlc;
    wire [63:0] tx_push_data;

    wire tx_fifo_pop = frame_good && is_transmitter; // design note 5

    can_frame_fifo #(.DEPTH(`CAN_FIFO_DEPTH)) u_tx_fifo (
        .clk(clk), .rst_n(rst_n),
        .push(tx_push), .id_in(tx_push_id), .ide_in(tx_push_ide),
        .rtr_in(tx_push_rtr), .dlc_in(tx_push_dlc), .data_in(tx_push_data),
        .pop(tx_fifo_pop),
        .id_out(tx_fifo_id_out), .ide_out(tx_fifo_ide_out), .rtr_out(tx_fifo_rtr_out),
        .dlc_out(tx_fifo_dlc_out), .data_out(tx_fifo_data_out),
        .full(tx_fifo_full), .empty(tx_fifo_empty),
        .empty_to_nonempty(tx_fifo_empty_to_nonempty),
        .full_to_nonfull(tx_fifo_full_to_nonfull)
    );

    wire ack_drive;
    wire ack_error_w;

    can_ack u_ack (
        .clk(clk), .rst_n(rst_n),
        .reset_ack(reset_ack), .ack_enable(ack_enable), .sample_point(sample_point),
        .is_transmitter(is_transmitter), .bus_bit(bus_bit),
        .ack_drive(ack_drive), .ack_error(ack_error_w)
    );
    assign ack_error = ack_error_w;

    // Byte/bit decomposition of field_bit_index within the Data
    // field, for the TX generator (raw_bit_tx below), which needs
    // the LIVE field_bit_index (it's read combinationally, same
    // edge as can_stuff_tx consumes it).
    wire [6:0] data_byte_idx    = field_bit_index >> 3;      // /8
    wire [2:0] data_bit_in_byte = field_bit_index[2:0];      // %8
    wire [6:0] data_abs_bit     = (data_byte_idx << 3) + (3'd7 - data_bit_in_byte);

    // Same decomposition for the RX capture logic below, but off
    // `field_bit_index_wire` (declared further down, alongside
    // `field_state_wire`) instead of the live `field_bit_index` -
    // see the bug #3 writeup on `field_state_wire` for why RX must
    // use the one-bit-period-delayed copy, not the live signal.
    wire [6:0] data_byte_idx_wire    = field_bit_index_wire >> 3;
    wire [2:0] data_bit_in_byte_wire = field_bit_index_wire[2:0];
    wire [6:0] data_abs_bit_wire     = (data_byte_idx_wire << 3) + (3'd7 - data_bit_in_byte_wire);

    reg raw_bit_tx;
    always @(*) begin
        case (field_state)
            `CAN_FLD_SOF:
                raw_bit_tx = is_transmitter ? `CAN_DOMINANT : `CAN_RECESSIVE;

            `CAN_FLD_ARBITRATION:
                raw_bit_tx = is_transmitter
                    ? ((field_bit_index < 7'd11) ? tx_fifo_id_out[7'd10 - field_bit_index]
                                                   : tx_fifo_rtr_out)
                    : `CAN_RECESSIVE;

            `CAN_FLD_CONTROL:
                raw_bit_tx = is_transmitter
                    ? ((field_bit_index == 7'd0) ? tx_fifo_ide_out :
                       (field_bit_index == 7'd1) ? `CAN_DOMINANT   : // r0, reserved
                                                    tx_fifo_dlc_out[7'd5 - field_bit_index])
                    : `CAN_RECESSIVE;

            `CAN_FLD_DATA:
                raw_bit_tx = is_transmitter ? tx_fifo_data_out[data_abs_bit] : `CAN_RECESSIVE;

            `CAN_FLD_CRC:
                raw_bit_tx = is_transmitter ? tx_crc_reg[7'd14 - field_bit_index] : `CAN_RECESSIVE;

            // Bug #4 (found immediately after bug #3 above was fixed and
            // crc_error finally stopped firing - the self-test then
            // started failing on ack_error instead, every frame): the
            // LOOPBACK auto-acknowledge (design note 3) originally forced
            // `raw_bit_tx` dominant here, on CAN_FLD_ACK_SLOT's own tick -
            // but `tx_bit_final` is can_stuff_tx's *registered* output
            // (see the field_state_wire/bug #3 writeup above, right after
            // u_stuff_tx), so a value forced here doesn't reach the wire
            // until the *next* bit period, which by then is CAN_FLD_
            // ACK_DELIM, not CAN_FLD_ACK_SLOT. The wire during the TRUE
            // ACK_SLOT bit period was actually left showing whatever
            // CAN_FLD_CRC_DELIM's tick had produced - unconditionally
            // recessive, before this fix carved CRC_DELIM out of the
            // catch-all `default` case below - so `can_ack` (unmodified,
            // and correctly reading the real, live wire at
            // the real `sample_point` - this was never its bug) sampled a
            // genuinely recessive ACK slot and correctly reported "no ack
            // seen". Confirmed with the same Python re-simulation
            // approach as bug #3: modeling both the original ACK_SLOT-
            // side forcing and this fix, only this fix produces a
            // dominant bit on the wire during the bit period that's
            // actually sampled as CAN_FLD_ACK_SLOT.
            //
            // Fix: force the bit one field *earlier* instead - on
            // CAN_FLD_CRC_DELIM's own tick (frame_fsm.v's CRC_DELIM->
            // ACK_SLOT transition is unconditional, always exactly one
            // bit long, so "current tick is CRC_DELIM" always means
            // "next tick is ACK_SLOT" and this can't skip or double up).
            // That's exactly one bit period early enough for can_stuff_tx's
            // own registration delay to land it on the wire during the
            // true ACK_SLOT period instead. CAN_FLD_ACK_SLOT's own case
            // reverts to plain `ack_drive` (no loopback special-case
            // needed here any more) - is_transmitter's `ack_drive` is
            // recessive, so the bit it now produces (which lands on the
            // wire during the true CAN_FLD_ACK_DELIM period) is the
            // correct recessive delimiter either way.
            `CAN_FLD_CRC_DELIM:
                raw_bit_tx = can_loopback ? `CAN_DOMINANT : `CAN_RECESSIVE;

            `CAN_FLD_ACK_SLOT:
                raw_bit_tx = ack_drive;

            `CAN_FLD_ERROR:
                // Any node that reaches the error field actively
                // drives the error flag - both roles participate,
                // matching real CAN error signaling.
                raw_bit_tx = (field_bit_index < 7'd6) ? `CAN_DOMINANT : `CAN_RECESSIVE;

            default: // ACK_DELIM, EOF, INTERMISSION, IDLE
                raw_bit_tx = `CAN_RECESSIVE;
        endcase
    end

    wire stuff_inserted_tx;
    wire stuff_now_tx;
    wire [2:0] same_count_tx_unused;

    can_stuff_tx u_stuff_tx (
        .clk(clk), .rst_n(rst_n),
        .stuff_enable(stuff_enable), .reset_stuff(reset_stuff),
        .bit_tick(sync_pulse), .raw_bit(raw_bit_tx),
        .tx_bit(tx_bit_final), .stuff_inserted(stuff_inserted_tx),
        .same_count(same_count_tx_unused),
        .stuff_now(stuff_now_tx)
    );

    // ------------------------------------------------------------
    // Bug #3 (found by clock-accurate Python re-simulation against
    // the golden CRC-15 reference, after bugs #1/#2 above were
    // fixed and the self-test *still* failed on crc_error every
    // time): `tx_bit_final` (above) is `can_stuff_tx`'s own
    // *registered* output - at each `sync_pulse`, it latches
    // `raw_bit_tx` as combinationally computed from THIS tick's
    // (pre-edge) `field_state`/`field_bit_index`, so the value that
    // lands on the wire only becomes visible starting the *next*
    // bit period. Put another way: the wire content actually
    // present during bit period N is whatever `field_state`/
    // `field_bit_index` said was current during bit period N-1 -
    // a full bit-period of structural lag between the frame FSM's
    // own bookkeeping and what's really on `can_tx`/`bus_bit`.
    //
    // The RX content-capture logic below (`rx_id_shift` etc. and
    // `rx_crc_shift_en`) receives its data (`rx_bit_d`) already
    // correctly delayed to match the wire (see the comment on
    // `rx_bit_d` below) - but it was bucketing that wire-accurate
    // data using the *live* `field_state`/`field_bit_index`, which
    // is one full bit period ahead of what the just-sampled bit
    // actually belongs to. Every field boundary crossed during
    // RX capture: the boundary's first wire bit still holds the
    // *previous* field's last content bit, live `field_state` says
    // otherwise, and the bit is bucketed one field early - which is
    // exactly why the destination register (rx_crc_reg here, but
    // also rx_id_shift/rx_dlc_shift/rx_data_shift, though those
    // happened not to be checked by name in this particular
    // scenario) came out corrupted.
    //
    // Fix: keep a registered copy of `field_state`/`field_bit_index`
    // that lags by exactly one bit period, updated at the same
    // `sync_pulse` edge and from the same pre-edge values that
    // `can_stuff_tx` itself used to produce `tx_bit_final` for this
    // period - i.e. built to mirror `tx_bit_final`'s own timing
    // exactly, so it always names the field that the CURRENT wire
    // content actually belongs to. Every RX-side field-membership
    // decision below (`rx_crc_shift_en`'s CRC-window gate and the
    // `field_state` case in the RX capture always block, including
    // the Data field's `rx_data_abs_bit`) uses this delayed copy,
    // not the live one.
    //
    // Verified with a full clock/TQ-accurate Python re-simulation
    // of this exact RTL (docs/progress_notes.md has the full
    // writeup): reproduced the DUT's real buggy values bit-for-bit
    // with the live field_state (rx_crc_reg=0x7d86, rx_crc_shift=
    // 0x34b5, rx_id_shift=0x122, rx_rtr_shift=1, rx_dlc_shift=4,
    // rx_data_shift=0x091a2b3cef56df77 for Scenario A's real frame,
    // ID=0x245/DLC=8/data=DEAD_BEEF_12345678) - none of those
    // matched the transmitted frame, confirming every RX-captured
    // field was corrupted by this bug, not just the CRC. With this
    // fix applied in the simulation, EVERY one of those registers
    // came out exactly matching the transmitted frame (rx_crc_reg=
    // rx_crc_shift=tx_crc_reg=0x696a, rx_id_shift=0x245, rx_rtr_
    // shift=0, rx_dlc_shift=8, rx_data_shift=0x12345678deadbeef) -
    // a full, independent, bit-exact match across every captured
    // field, not just the one (CRC) this bug happened to be found
    // through.
    // ------------------------------------------------------------
    // (field_state_wire/field_bit_index_wire themselves are declared
    // earlier, in the forward-declarations block, alongside
    // data_abs_bit_wire's other users.)
    always @(posedge clk) begin
        if (!rst_n) begin
            field_state_wire     <= `CAN_FLD_IDLE;
            field_bit_index_wire <= 7'd0;
        end
        else if (sync_pulse) begin
            field_state_wire     <= field_state;
            field_bit_index_wire <= field_bit_index;
        end
    end

    // tx_crc_reg is declared earlier, in the forward-declarations
    // block above.
    //
    // Bug found at Step 9b full-system sim (self-test always aborted
    // with crc_error, every frame, every scenario): this MUST gate
    // off `stuff_now_tx` (can_stuff_tx's same-edge combinational
    // "this tick is a forced stuff bit" output), not the registered
    // `stuff_inserted_tx`. `stuff_inserted_tx` only reflects *last*
    // tick's stuffing outcome (it can't be visible any earlier - it's
    // a clocked output of the same always block that decides it), so
    // gating this tick's `raw_bit_tx` read with it silently shifted
    // the CRC accumulator on the wrong ticks (dropping the real
    // content bit that follows a stuff bit, re-shifting a stale one
    // instead) any time a frame contained bit stuffing anywhere in
    // SOF..Data - which is effectively every real frame. The RX side
    // (`rx_crc_shift_en` below) doesn't have this problem: its gate
    // (`valid_bit_rx`) and its data (`rx_bit_d`) are BOTH registered
    // with the same one-tick delay, so they stay in lockstep; here on
    // TX, the gate was registered but `raw_bit_tx` is combinational,
    // so the two drifted a tick apart.
    wire tx_crc_shift_en = sync_pulse && !stuff_now_tx && crc_enable;

    can_crc15 u_crc_tx (
        .clk(clk), .rst_n(rst_n),
        .reset_crc(reset_crc), .shift_en(tx_crc_shift_en), .data_bit(raw_bit_tx),
        .crc_reg(tx_crc_reg)
    );

    // ------------------------------------------------------------
    // Arbitration - my_bit is the actual driven wire bit
    // (post-stuffing). When is_transmitter is 0 this is a constant
    // recessive stream (occasionally forced dominant by our own
    // stuffing on a long recessive run), so arbitration_lost can in
    // principle pulse even though this node was never contending -
    // harmless: frame_fsm's only reaction is `is_transmitter<=0`,
    // which is already 0, a no-op. Documented rather than
    // specially gated, to keep this integration simple.
    // ------------------------------------------------------------
    wire lost_latched_unused;

    can_arbitration u_arb (
        .clk(clk), .rst_n(rst_n),
        .reset_arb(reset_arb), .arb_enable(arb_enable), .sample_point(sample_point),
        .my_bit(tx_bit_final), .bus_bit(bus_bit),
        .arbitration_lost(arbitration_lost), .lost_latched(lost_latched_unused)
    );

    // ------------------------------------------------------------
    // RX side: destuffer -> content capture shift registers -> RX FIFO
    // ------------------------------------------------------------
    wire valid_bit_rx, stuff_err_rx;
    assign stuff_error_rx = stuff_err_rx;
    wire [2:0] same_count_rx_unused;

    can_destuff_rx u_destuff_rx (
        .clk(clk), .rst_n(rst_n),
        .stuff_enable(stuff_enable), .reset_stuff(reset_stuff),
        .bit_tick(sample_point), .rx_bit(bus_bit),
        .valid_bit(valid_bit_rx), .same_count(same_count_rx_unused),
        .stuff_error(stuff_err_rx)
    );

    // Capture the bus level AT the sample_point instant, so it is
    // still available (rx_bit_d) the cycle after, in lockstep with
    // valid_bit's own one-cycle-registered latency.
    reg rx_bit_d;
    always @(posedge clk) begin
        if (!rst_n) rx_bit_d <= `CAN_RECESSIVE;
        else if (sample_point) rx_bit_d <= bus_bit;
    end

    // stuff_hold reconstruction for the RX role: valid_bit/
    // stuff_error are one-shot pulses appearing exactly one clock
    // after sample_point (see can_destuff_rx.v). Latch "this bit
    // was a discarded stuff bit" at that moment and hold it stable
    // until the next sync_pulse consumes it (comfortably before the
    // following sample_point, for any realistic BTIME config).
    reg sample_point_d1;
    always @(posedge clk) begin
        if (!rst_n) sample_point_d1 <= 1'b0;
        else        sample_point_d1 <= sample_point;
    end

    reg rx_stuff_hold_latched;
    always @(posedge clk) begin
        if (!rst_n || reset_stuff)
            rx_stuff_hold_latched <= 1'b0;
        else if (sample_point_d1)
            rx_stuff_hold_latched <= (!valid_bit_rx && !stuff_err_rx);
    end

    // Bug found alongside the tx_crc_shift_en fix above, same root
    // cause, confirmed by a cycle-accurate Python re-simulation of
    // this exact logic against the golden CRC-15 reference (matched
    // the DUT's buggy tx_crc_reg=0x784a bit-for-bit with the stale
    // gate, and the golden 0x696a with this fix): `stuff_hold` on
    // the transmitter side must NOT use `stuff_inserted_tx` either,
    // for exactly the same reason - it's a *registered* output that
    // can only reflect *last* tick's stuffing decision, one full bit
    // period stale relative to what `field_bit_index` needs THIS
    // tick. can_frame_fsm.v uses `stuff_hold` to decide whether to
    // freeze `field_bit_index` so a content bit deferred by a forced
    // stuff bit gets re-presented on the very next tick - with the
    // stale signal, the freeze fires one tick too late: by the time
    // `stuff_hold` finally reads 1, `field_bit_index` has *already*
    // advanced past the deferred bit (using the previous tick's now-
    // irrelevant stuffing outcome), permanently skipping it and
    // shifting every subsequent logical bit one position early. Using
    // `stuff_now_tx` (the same same-edge combinational signal the CRC
    // gate above now uses) keeps the freeze decision on the correct
    // edge. The RX branch (`rx_stuff_hold_latched`) does NOT have this
    // problem - it's deliberately latched at `sample_point_d1`, well
    // before the *next* sync_pulse, so by construction it already
    // reflects the just-completed bit's stuffing status in time for
    // frame_fsm's read; only the TX branch needed to change.
    assign stuff_hold = is_transmitter ? stuff_now_tx : rx_stuff_hold_latched;

    wire [14:0] rx_crc_reg;

    // Bug #3 fix: gate on the delayed `field_state_wire`'s window
    // (SOF..end of Data, mirroring `crc_enable`'s own definition in
    // can_frame_fsm.v exactly, just off the wire-accurate copy of
    // field_state instead of the live one), NOT the live `crc_enable`
    // wire from the frame FSM - see the writeup on `field_state_wire`
    // above (right after u_stuff_tx) for why.
    wire rx_crc_enable_wire =
        (field_state_wire == `CAN_FLD_SOF)         ||
        (field_state_wire == `CAN_FLD_ARBITRATION) ||
        (field_state_wire == `CAN_FLD_CONTROL)     ||
        (field_state_wire == `CAN_FLD_DATA);

    wire rx_crc_shift_en = valid_bit_rx && rx_crc_enable_wire;

    can_crc15 u_crc_rx (
        .clk(clk), .rst_n(rst_n),
        .reset_crc(reset_crc), .shift_en(rx_crc_shift_en), .data_bit(rx_bit_d),
        .crc_reg(rx_crc_reg)
    );

    // RX content capture: ID/RTR (Arbitration), IDE/DLC (Control),
    // Data (byte/bit-addressed, same formula as the TX generator),
    // and the received CRC field bits (for comparison against
    // rx_crc_reg, computed independently over SOF..Data only).
    // rx_id_shift/rx_rtr_shift/rx_ide_shift/rx_dlc_shift/
    // rx_data_shift/rx_crc_shift are declared earlier, in the
    // forward-declarations block above (rx_dlc_shift specifically
    // needs to be visible before the u_frame_fsm instantiation,
    // which feeds it in as rx_dlc_capture).

    always @(posedge clk) begin
        if (!rst_n || reset_stuff) begin
            rx_id_shift   <= 11'd0;
            rx_rtr_shift  <= 1'b0;
            rx_ide_shift  <= 1'b0;
            rx_dlc_shift  <= 4'd0;
            rx_data_shift <= 64'd0;
            rx_crc_shift  <= 15'd0;
        end
        else if (valid_bit_rx) begin
            // Bucketed by field_state_wire/field_bit_index_wire (the
            // one-bit-period-delayed copy), not the live field_state/
            // field_bit_index - see the bug #3 writeup above (right
            // after u_stuff_tx) for why: rx_bit_d is already
            // wire-accurate, but live field_state is one bit period
            // ahead of what that wire content actually belongs to.
            case (field_state_wire)
                `CAN_FLD_ARBITRATION: begin
                    if (field_bit_index_wire < 7'd11)
                        rx_id_shift <= {rx_id_shift[9:0], rx_bit_d};
                    else
                        rx_rtr_shift <= rx_bit_d;
                end

                `CAN_FLD_CONTROL: begin
                    if (field_bit_index_wire == 7'd0)
                        rx_ide_shift <= rx_bit_d;
                    else if (field_bit_index_wire != 7'd1) // skip r0
                        rx_dlc_shift <= {rx_dlc_shift[2:0], rx_bit_d};
                end

                `CAN_FLD_DATA: begin
                    rx_data_shift[data_abs_bit_wire] <= rx_bit_d;
                end

                `CAN_FLD_CRC: begin
                    rx_crc_shift <= {rx_crc_shift[13:0], rx_bit_d};
                end

                default: ; // no capture in other fields
            endcase
        end
    end

    assign crc_error = (rx_crc_shift != rx_crc_reg);

    // ------------------------------------------------------------
    // RX FIFO
    // ------------------------------------------------------------
    wire [28:0] rx_fifo_id_out;
    wire        rx_fifo_ide_out, rx_fifo_rtr_out;
    wire [3:0]  rx_fifo_dlc_out;
    wire [63:0] rx_fifo_data_out;
    wire        rx_fifo_full, rx_fifo_empty;
    wire        rx_fifo_empty_to_nonempty, rx_fifo_full_to_nonfull;

    wire rx_fifo_push = frame_good && (!is_transmitter || can_loopback); // design note 4
    wire rx_pop;

    can_frame_fifo #(.DEPTH(`CAN_FIFO_DEPTH)) u_rx_fifo (
        .clk(clk), .rst_n(rst_n),
        .push(rx_fifo_push),
        .id_in({18'd0, rx_id_shift}), .ide_in(rx_ide_shift),
        .rtr_in(rx_rtr_shift), .dlc_in(rx_dlc_shift), .data_in(rx_data_shift),
        .pop(rx_pop),
        .id_out(rx_fifo_id_out), .ide_out(rx_fifo_ide_out), .rtr_out(rx_fifo_rtr_out),
        .dlc_out(rx_fifo_dlc_out), .data_out(rx_fifo_data_out),
        .full(rx_fifo_full), .empty(rx_fifo_empty),
        .empty_to_nonempty(rx_fifo_empty_to_nonempty),
        .full_to_nonfull(rx_fifo_full_to_nonfull)
    );

    // ------------------------------------------------------------
    // Error state machine
    // ------------------------------------------------------------
    wire [7:0] tec, rec;
    wire [1:0] err_state;
    wire err_state_changed;

    can_error_sm u_err_sm (
        .clk(clk), .rst_n(rst_n), .can_enable(can_en),
        .frame_good(frame_good), .frame_error(frame_error), .is_transmitter(is_transmitter),
        .sample_point(sample_point), .bus_bit(bus_bit),
        .tec(tec), .rec(rec), .err_state(err_state), .err_state_changed(err_state_changed)
    );

    // ------------------------------------------------------------
    // Interrupt pending-set pulses
    // ------------------------------------------------------------
    wire rx_ip_set  = rx_fifo_empty_to_nonempty;
    wire tx_ip_set  = frame_good && is_transmitter;
    wire err_ip_set = err_state_changed &&
                       ((err_state == `CAN_ERR_PASSIVE) || (err_state == `CAN_ERR_BUSOFF));

    // ------------------------------------------------------------
    // Register file
    // ------------------------------------------------------------
    can_regfile u_regfile (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .rdata(rdata),
        .we(we), .re(re), .sel(sel), .ready(ready),
        .can_en(can_en), .can_loopback(can_loopback), .soft_reset(soft_reset),
        .btime_brp(btime_brp), .btime_prop_seg(btime_prop_seg),
        .btime_phase_seg1(btime_phase_seg1), .btime_phase_seg2(btime_phase_seg2),
        .btime_sjw(btime_sjw),
        .tx_fifo_full(tx_fifo_full), .tx_fifo_empty(tx_fifo_empty),
        .rx_fifo_full(rx_fifo_full), .rx_fifo_empty(rx_fifo_empty),
        .err_state(err_state),
        .tx_push(tx_push), .tx_push_id(tx_push_id), .tx_push_ide(tx_push_ide),
        .tx_push_rtr(tx_push_rtr), .tx_push_dlc(tx_push_dlc), .tx_push_data(tx_push_data),
        .rx_pop(rx_pop),
        .rx_id(rx_fifo_id_out), .rx_ide(rx_fifo_ide_out), .rx_rtr(rx_fifo_rtr_out),
        .rx_dlc(rx_fifo_dlc_out), .rx_data(rx_fifo_data_out),
        .rx_ip_set(rx_ip_set), .tx_ip_set(tx_ip_set), .err_ip_set(err_ip_set),
        .irq_rx(irq_rx), .irq_tx(irq_tx), .irq_err(irq_err),
        .tec(tec), .rec(rec)
    );

    // soft_reset (CAN_CTRL.RESET) is a documented pulse output for
    // Person 2/3's reference; no additional frame-level state exists
    // at this integration layer beyond what can_enable already
    // resets in every sub-module (can_en gates or resets essentially
    // everything above via each module's own `can_enable`/`rst_n`
    // handling), so soft_reset is intentionally left unconnected to
    // any extra logic here.

endmodule
