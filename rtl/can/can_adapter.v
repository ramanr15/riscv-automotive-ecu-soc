`timescale 1ns / 1ps
// ============================================================
// can_adapter
//
// Thin boundary shim between Person 1's can_controller_top and
// Person 2's peripheral bus. It exists to reconcile two real
// mismatches without editing either side's RTL.
//
// ------------------------------------------------------------
// MISMATCH 1 : reset polarity
//
//   The SoC uses synchronous active-high `reset`.
//   can_controller_top uses active-low `rst_n`, per the WBS
//   Section 1.4.1 contract.
//
//   A bare inverter would work logically but releases the CAN
//   controller from reset on a combinational path off the
//   global reset net. This synchronises the inverted reset to
//   the system clock instead, so the CAN block leaves reset a
//   deterministic two cycles after everything else -- harmless,
//   and it keeps the reset a proper synchronous signal.
//
// ------------------------------------------------------------
// MISMATCH 2 : combinational read data
//
//   can_regfile.v drives `rdata` from an `always @(*)` block,
//   so read data is valid in the SAME cycle as the request.
//   The SoC bus contract (CAN_INTEGRATION_SPEC.md Rule 1) is
//   registered: request on cycle N, data on cycle N+1. The CPU
//   has no memory-stall path and cannot wait, and
//   bus_interconnect.v selects each slave's read data one cycle
//   late via sel_q. Wired directly, every CAN register read
//   would return the wrong word.
//
//   This shim registers `rdata` on the request cycle, which
//   converts the timing to the required contract.
//
//   The read-side-effect on CAN_RX_CTRL still behaves. In
//   can_regfile.v, `rx_pop = rd && (idx == IDX_RX_CTRL)` is a
//   combinational strobe: the FIFO advances on the clock edge
//   that ENDS cycle N, so during cycle N `rdata` still shows
//   the pre-pop head. Capturing at that same edge therefore
//   latches exactly the word being popped -- one pop, one
//   value, no loss.
//
//   This is a workaround at the boundary, not a fix. The proper
//   correction is in can_regfile.v (clocked read mux), which
//   also removes a 32-bit combinational mux from the bus path.
//   Person 1 has been asked to make that change; when it lands,
//   set REGISTER_RDATA = 0 and this shim becomes a pass-through.
//
// ------------------------------------------------------------
// The `ready` output of can_controller_top is tied to `sel`
// (no wait states), so it is intentionally left unconnected --
// the SoC bus has no ready path.
// ============================================================
module can_adapter #(
    // 1 = register rdata here (needed while can_regfile.v reads
    //     combinationally). 0 = pass through, once Person 1's
    //     regfile does its own registering.
    parameter REGISTER_RDATA = 1
)(
    input  wire        clk,
    input  wire        reset,        // synchronous, active high

    // ---- SoC bus slave port ----
    input  wire        sel,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output wire [31:0] prdata,

    // ---- Interrupts ----
    output wire        irq_rx,
    output wire        irq_tx,
    output wire        irq_err,

    // ---- CAN pads ----
    input  wire        can_rx,
    output wire        can_tx
);

    // ------------------------------------------------------------
    // Reset polarity conversion, synchronised
    // ------------------------------------------------------------
    reg rst_n_meta;
    reg rst_n_sync;

    always @(posedge clk) begin
        if (reset) begin
            rst_n_meta <= 1'b0;
            rst_n_sync <= 1'b0;
        end
        else begin
            rst_n_meta <= 1'b1;
            rst_n_sync <= rst_n_meta;
        end
    end

    // ------------------------------------------------------------
    // Person 1's controller
    // ------------------------------------------------------------
    wire [31:0] can_rdata_comb;

    can_controller_top U_CAN (
        .clk    (clk),
        .rst_n  (rst_n_sync),

        .addr   (addr),
        .wdata  (wdata),
        .rdata  (can_rdata_comb),
        .we     (we),
        .re     (re),
        .sel    (sel),
        .ready  (),          // tied to sel inside; unused here

        .can_tx (can_tx),
        .can_rx (can_rx),

        .irq_rx (irq_rx),
        .irq_tx (irq_tx),
        .irq_err(irq_err)
    );

    // ------------------------------------------------------------
    // Read-data timing conversion
    // ------------------------------------------------------------
    generate
        if (REGISTER_RDATA) begin : RDATA_REGISTERED
            reg [31:0] prdata_r;

            always @(posedge clk) begin
                if (reset)
                    prdata_r <= 32'b0;
                else if (sel && re)
                    prdata_r <= can_rdata_comb;
            end

            assign prdata = prdata_r;
        end
        else begin : RDATA_PASSTHROUGH
            assign prdata = can_rdata_comb;
        end
    endgenerate

endmodule
