`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// can_regfile.v
//
// WBS Section 2.3.10 - Step 9 (register half): the memory-mapped
// register file, per the bus convention in Section 1.4.1
// (`clk, rst_n, addr[31:0], wdata[31:0], rdata[31:0], we, re, sel,
// ready`) and the register map this module IS the authoritative
// implementation of (see docs/can_register_map.md for the
// per-register bit tables).
//
// Scope: this module is pure bus/register glue - it holds no
// protocol state of its own (no FIFOs, no frame sequencing). It
// exposes clean pulse/level interfaces that can_controller_top.v
// (the other half of Step 9) wires to the actual TX/RX FIFOs and
// the rest of the datapath. This split keeps the bus-facing
// register semantics (write-1-to-clear, W-only registers reading
// as 0, single-cycle no-wait-state transactions) independently
// testable without needing the full bit-level protocol stack
// running.
//
// Bus timing: every transaction completes in the cycle it is
// selected - `ready` is simply `sel` (no wait states needed for a
// register file this simple). Writes take effect combinationally-
// gated, registered on the same clock edge as the write; reads are
// combinational off `addr` (valid whenever `sel && re`).
//
// TX push / RX pop timing: `tx_push` and `rx_pop` are plain
// combinational decodes of "this cycle's bus write/read is to
// CAN_TX_CTRL / CAN_RX_CTRL" - not registered pulses - so they land
// on the exact same clock edge as the triggering bus transaction,
// matching can_frame_fifo.v's own same-edge `do_push`/`do_pop`
// contract. `tx_push_id`/`tx_push_data` come from the CAN_TX_ID/
// CAN_TX_DATA0/1 shadow registers latched on their own earlier
// writes; `tx_push_dlc`/`rtr`/`ide` come straight off this cycle's
// `wdata` (all three are written together with the CAN_TX_CTRL
// write, per Section 2.4.2's driver pattern: ID and DATA are
// written first, then CTRL is "strobed").
// ============================================================

module can_regfile (
    input  wire        clk,
    input  wire        rst_n,

    // ---- Bus (Section 1.4.1) ----
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    input  wire        we,
    input  wire        re,
    input  wire        sel,
    output wire         ready,

    // ---- CAN_CTRL (0x00) ----
    output wire        can_en,        // bit 0
    output wire        can_loopback,  // bit 1
    output wire        soft_reset,    // pulse: bit 2 written 1 (frame-level reset only)

    // ---- CAN_BTIME (0x04) ----
    output wire [7:0]  btime_brp,
    output wire [3:0]  btime_prop_seg,
    output wire [3:0]  btime_phase_seg1,
    output wire [3:0]  btime_phase_seg2,
    output wire [1:0]  btime_sjw,

    // ---- CAN_STATUS (0x08) inputs ----
    input  wire        tx_fifo_full,
    input  wire        tx_fifo_empty,
    input  wire        rx_fifo_full,
    input  wire        rx_fifo_empty,
    input  wire [1:0]  err_state,

    // ---- TX push interface (CAN_TX_ID/DATA0/DATA1/CTRL, 0x0C-0x18) ----
    output wire         tx_push,       // pulse, same edge as the CAN_TX_CTRL write
    output wire [28:0]  tx_push_id,
    output wire         tx_push_ide,
    output wire         tx_push_rtr,
    output wire [3:0]   tx_push_dlc,
    output wire [63:0]  tx_push_data,

    // ---- RX pop interface (CAN_RX_ID/DATA0/DATA1/CTRL, 0x1C-0x28) ----
    output wire         rx_pop,        // pulse, same edge as the CAN_RX_CTRL read
    input  wire [28:0]  rx_id,         // head-of-FIFO, combinational (Step 2 contract)
    input  wire         rx_ide,
    input  wire         rx_rtr,
    input  wire [3:0]   rx_dlc,
    input  wire [63:0]  rx_data,

    // ---- Interrupts (CAN_IE 0x2C, CAN_IP 0x30) ----
    input  wire         rx_ip_set,     // pulse: frame arrived in RX FIFO
    input  wire         tx_ip_set,     // pulse: a TX frame completed
    input  wire         err_ip_set,    // pulse: err_state entered Passive/Bus-Off
    output wire         irq_rx,        // level: masked pending (IRQ ID 0, Section 1.4.3)
    output wire         irq_tx,        // level: masked pending (IRQ ID 1)
    output wire         irq_err,       // level: masked pending (IRQ ID 2)

    // ---- CAN_TEC_REC (0x34) inputs ----
    input  wire [7:0]   tec,
    input  wire [7:0]   rec
);

    assign ready = sel;

    // ------------------------------------------------------------
    // Word-aligned register index (offsets 0x00-0x34, 4-byte words)
    // ------------------------------------------------------------
    localparam [5:0] IDX_CTRL     = 6'd0;  // 0x00
    localparam [5:0] IDX_BTIME    = 6'd1;  // 0x04
    localparam [5:0] IDX_STATUS   = 6'd2;  // 0x08
    localparam [5:0] IDX_TX_ID    = 6'd3;  // 0x0C
    localparam [5:0] IDX_TX_DATA0 = 6'd4;  // 0x10
    localparam [5:0] IDX_TX_DATA1 = 6'd5;  // 0x14
    localparam [5:0] IDX_TX_CTRL  = 6'd6;  // 0x18
    localparam [5:0] IDX_RX_ID    = 6'd7;  // 0x1C
    localparam [5:0] IDX_RX_DATA0 = 6'd8;  // 0x20
    localparam [5:0] IDX_RX_DATA1 = 6'd9;  // 0x24
    localparam [5:0] IDX_RX_CTRL  = 6'd10; // 0x28
    localparam [5:0] IDX_IE       = 6'd11; // 0x2C
    localparam [5:0] IDX_IP       = 6'd12; // 0x30
    localparam [5:0] IDX_TEC_REC  = 6'd13; // 0x34

    wire [5:0] idx = addr[7:2];

    wire wr = sel && we;
    wire rd = sel && re;

    // ------------------------------------------------------------
    // CAN_CTRL - EN/LOOPBACK stored; RESET (bit 2) is a pure
    // write-triggered pulse, never stored, always reads back 0.
    // ------------------------------------------------------------
    reg [1:0] ctrl_reg; // [0]=EN, [1]=LOOPBACK

    assign can_en       = ctrl_reg[0];
    assign can_loopback = ctrl_reg[1];
    assign soft_reset   = wr && (idx == IDX_CTRL) && wdata[2];

    always @(posedge clk) begin
        if (!rst_n)
            ctrl_reg <= 2'b00;
        else if (wr && (idx == IDX_CTRL))
            ctrl_reg <= wdata[1:0];
    end

    // ------------------------------------------------------------
    // CAN_BTIME - stored as one 32-bit word, sliced per the layout
    // in docs/can_register_map.md.
    // ------------------------------------------------------------
    reg [31:0] btime_reg;

    assign btime_brp        = btime_reg[7:0];
    assign btime_prop_seg   = btime_reg[11:8];
    assign btime_phase_seg1 = btime_reg[15:12];
    assign btime_phase_seg2 = btime_reg[19:16];
    assign btime_sjw        = btime_reg[21:20];

    always @(posedge clk) begin
        if (!rst_n)
            btime_reg <= 32'd0;
        else if (wr && (idx == IDX_BTIME))
            btime_reg <= wdata;
    end

    // ------------------------------------------------------------
    // CAN_TX_ID / CAN_TX_DATA0 / CAN_TX_DATA1 - shadow registers,
    // latched on their own writes, consumed combinationally when
    // CAN_TX_CTRL is written (the "strobe").
    // ------------------------------------------------------------
    reg [28:0] tx_id_reg;
    reg [31:0] tx_data0_reg;
    reg [31:0] tx_data1_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            tx_id_reg    <= 29'd0;
            tx_data0_reg <= 32'd0;
            tx_data1_reg <= 32'd0;
        end
        else begin
            if (wr && (idx == IDX_TX_ID))    tx_id_reg    <= wdata[28:0];
            if (wr && (idx == IDX_TX_DATA0)) tx_data0_reg <= wdata;
            if (wr && (idx == IDX_TX_DATA1)) tx_data1_reg <= wdata;
        end
    end

    assign tx_push      = wr && (idx == IDX_TX_CTRL);
    assign tx_push_dlc  = wdata[3:0];
    assign tx_push_rtr  = wdata[4];
    assign tx_push_ide  = wdata[5];
    assign tx_push_id   = tx_id_reg;
    assign tx_push_data = {tx_data1_reg, tx_data0_reg};

    // ------------------------------------------------------------
    // RX pop - reading CAN_RX_CTRL is the pop strobe (Section
    // 2.4.2); CAN_RX_ID/DATA0/DATA1 are the FIFO's continuously-
    // exposed head entry, read combinationally, so software reading
    // them *before* CAN_RX_CTRL sees the entry this pop is about to
    // remove - the ordering the driver pattern relies on.
    // ------------------------------------------------------------
    assign rx_pop = rd && (idx == IDX_RX_CTRL);

    // ------------------------------------------------------------
    // CAN_IE / CAN_IP - enable mask and write-1-to-clear pending
    // flags. A same-cycle set and clear on the same bit favors the
    // set (a real event is never silently dropped by a clear that
    // happened to land on the same edge).
    // ------------------------------------------------------------
    reg [2:0] ie_reg;
    reg [2:0] ip_reg;

    wire ip_wr = wr && (idx == IDX_IP);

    wire next_ip_rx  = rx_ip_set  ? 1'b1 : ((ip_wr && wdata[`CAN_INT_RX_BIT])  ? 1'b0 : ip_reg[`CAN_INT_RX_BIT]);
    wire next_ip_tx  = tx_ip_set  ? 1'b1 : ((ip_wr && wdata[`CAN_INT_TX_BIT])  ? 1'b0 : ip_reg[`CAN_INT_TX_BIT]);
    wire next_ip_err = err_ip_set ? 1'b1 : ((ip_wr && wdata[`CAN_INT_ERR_BIT]) ? 1'b0 : ip_reg[`CAN_INT_ERR_BIT]);

    always @(posedge clk) begin
        if (!rst_n) begin
            ie_reg <= 3'd0;
            ip_reg <= 3'd0;
        end
        else begin
            if (wr && (idx == IDX_IE)) ie_reg <= wdata[2:0];

            ip_reg[`CAN_INT_RX_BIT]  <= next_ip_rx;
            ip_reg[`CAN_INT_TX_BIT]  <= next_ip_tx;
            ip_reg[`CAN_INT_ERR_BIT] <= next_ip_err;
        end
    end

    assign irq_rx  = ip_reg[`CAN_INT_RX_BIT]  && ie_reg[`CAN_INT_RX_BIT];
    assign irq_tx  = ip_reg[`CAN_INT_TX_BIT]  && ie_reg[`CAN_INT_TX_BIT];
    assign irq_err = ip_reg[`CAN_INT_ERR_BIT] && ie_reg[`CAN_INT_ERR_BIT];

    // ------------------------------------------------------------
    // Read mux - combinational, per docs/can_register_map.md.
    // W-only registers (TX_ID/DATA0/DATA1/CTRL) and any unmapped
    // address read as 0, per Section 1.4.2's "unused bits read as
    // zero" convention extended to whole unmapped/write-only words.
    // ------------------------------------------------------------
    always @(*) begin
        case (idx)
            IDX_CTRL:     rdata = {29'd0, 1'b0, ctrl_reg};           // bit2 (RESET) always reads 0
            IDX_BTIME:    rdata = {10'd0, btime_sjw, btime_phase_seg2,
                                    btime_phase_seg1, btime_prop_seg, btime_brp};
            IDX_STATUS:   rdata = {26'd0, err_state, rx_fifo_empty, rx_fifo_full,
                                    tx_fifo_empty, tx_fifo_full};
            IDX_RX_ID:    rdata = {3'd0, rx_id};
            IDX_RX_DATA0: rdata = rx_data[31:0];
            IDX_RX_DATA1: rdata = rx_data[63:32];
            IDX_RX_CTRL:  rdata = {26'd0, rx_ide, rx_rtr, rx_dlc};
            IDX_IE:       rdata = {29'd0, ie_reg};
            IDX_IP:       rdata = {29'd0, ip_reg};
            IDX_TEC_REC:  rdata = {16'd0, rec, tec};
            default:      rdata = 32'd0; // TX_ID/DATA0/DATA1/CTRL (W-only) + unmapped
        endcase
    end

endmodule
