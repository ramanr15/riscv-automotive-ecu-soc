`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_controller_top.v
//
// Step 9b self-check + Section 2.5's required loopback self-test:
// drives ONLY the memory-mapped bus (exactly as the CPU driver
// would - CAN_CTRL/CAN_BTIME configuration, CAN_TX_* pushes,
// CAN_RX_*/CAN_STATUS/CAN_IE/CAN_IP/CAN_TEC_REC reads), with
// CAN_CTRL.LOOPBACK=1, and confirms a frame pushed on the TX side
// comes back out correctly on the RX side after the full bit-level
// encode -> wire -> decode round trip through every Step 1-9a
// sub-block. This is the first test that exercises all of them
// together; every sub-block already has its own standalone
// testbench from its own Step.
//
// Completion is detected by POLLING CAN_STATUS.RX_FIFO_EMPTY over
// the bus (with a generous cycle-count guard) rather than by
// hand-computing an exact expected latency - the same defensive
// "guarded wait" pattern already used by tb_can_frame_fsm.v's
// advance_to task, and far more robust here given how many bit
// times (with variable stuff-bit overhead) a full frame spans.
// ============================================================

module tb_can_controller_top;

    reg clk, rst_n;
    reg  [31:0] addr, wdata;
    wire [31:0] rdata;
    reg  we, re, sel;
    wire ready;

    reg  can_rx;
    wire can_tx;
    wire irq_rx, irq_tx, irq_err;

    integer errors;
    reg [31:0] rd_result;

    can_controller_top DUT (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .rdata(rdata),
        .we(we), .re(re), .sel(sel), .ready(ready),
        .can_tx(can_tx), .can_rx(can_rx),
        .irq_rx(irq_rx), .irq_tx(irq_tx), .irq_err(irq_err)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------
    // TEMPORARY debug instrumentation - not part of the pass/fail
    // logic. Prints every field_state transition (via hierarchical
    // reference into the DUT) and every frame_good/frame_error
    // pulse with the abort-condition signals live at that instant,
    // so we can see exactly where/why a frame is aborting instead
    // of guessing from outside. Field encoding (can_defs.v):
    // 0=IDLE 1=SOF 2=ARBITRATION 3=CONTROL 4=DATA 5=CRC 6=CRC_DELIM
    // 7=ACK_SLOT 8=ACK_DELIM 9=EOF 10=INTERMISSION 11=ERROR
    // ------------------------------------------------------------
    reg [3:0] dbg_field_state_prev;
    initial dbg_field_state_prev = 4'hF; // force a print on the first real change

    // Counts `stuff_now` pulses (the TX stuffer's same-edge "forcing a
    // stuff bit this tick" signal) since the last SOF, so we can see
    // directly whether a given frame attempt actually contained any
    // bit stuffing at all - resets on reset_stuff (asserted at SOF).
    integer dbg_stuff_count;
    initial dbg_stuff_count = 0;

    always @(posedge clk) begin
        if (DUT.reset_stuff)
            dbg_stuff_count <= 0;
        else if (DUT.stuff_now_tx)
            dbg_stuff_count <= dbg_stuff_count + 1;

        if (DUT.field_state !== dbg_field_state_prev) begin
            $display("[DBG %0t] field_state %0d -> %0d  is_transmitter=%0d tx_request=%0d field_bit_index=%0d tec=%0d rec=%0d stuff_count=%0d",
                      $time, dbg_field_state_prev, DUT.field_state,
                      DUT.is_transmitter, DUT.tx_request, DUT.field_bit_index,
                      DUT.tec, DUT.rec, dbg_stuff_count);
            dbg_field_state_prev <= DUT.field_state;
        end
        if (DUT.frame_error)
            $display("[DBG %0t] *** frame_error *** field_state=%0d crc_error=%0d ack_error=%0d stuff_error_rx=%0d arbitration_lost=%0d stuff_count=%0d tx_crc_reg=%0h rx_crc_reg=%0h rx_crc_shift=%0h",
                      $time, DUT.field_state, DUT.crc_error, DUT.ack_error,
                      DUT.stuff_error_rx, DUT.arbitration_lost, dbg_stuff_count,
                      DUT.tx_crc_reg, DUT.rx_crc_reg, DUT.rx_crc_shift);
        if (DUT.frame_good)
            $display("[DBG %0t] *** frame_good ***", $time);
    end

    // ------------------------------------------------------------
    // Register offsets (docs/can_register_map.md)
    // ------------------------------------------------------------
    localparam [31:0] ADDR_CTRL     = 32'h00;
    localparam [31:0] ADDR_BTIME    = 32'h04;
    localparam [31:0] ADDR_STATUS   = 32'h08;
    localparam [31:0] ADDR_TX_ID    = 32'h0C;
    localparam [31:0] ADDR_TX_DATA0 = 32'h10;
    localparam [31:0] ADDR_TX_DATA1 = 32'h14;
    localparam [31:0] ADDR_TX_CTRL  = 32'h18;
    localparam [31:0] ADDR_RX_ID    = 32'h1C;
    localparam [31:0] ADDR_RX_DATA0 = 32'h20;
    localparam [31:0] ADDR_RX_DATA1 = 32'h24;
    localparam [31:0] ADDR_RX_CTRL  = 32'h28;
    localparam [31:0] ADDR_IE       = 32'h2C;
    localparam [31:0] ADDR_IP       = 32'h30;
    localparam [31:0] ADDR_TEC_REC  = 32'h34;

    task check;
        input cond;
        input [255:0] name;
        begin
            if (!cond) begin
                $display("FAIL: %0s @ time %0t", name, $time);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s @ time %0t", name, $time);
            end
        end
    endtask

    task bus_write;
        input [31:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            addr = a; wdata = d; we = 1'b1; re = 1'b0; sel = 1'b1;
            @(posedge clk); #1;
            @(negedge clk);
            we = 1'b0; sel = 1'b0;
        end
    endtask

    task bus_read;
        input [31:0] a;
        begin
            @(negedge clk);
            addr = a; we = 1'b0; re = 1'b1; sel = 1'b1;
            #1;
            rd_result = rdata;
            @(posedge clk);
            @(negedge clk);
            re = 1'b0; sel = 1'b0;
        end
    endtask

    // Poll CAN_STATUS until RX_FIFO_EMPTY (bit 3) reads 0, i.e. a
    // frame has arrived. Does one read unconditionally before the
    // loop guard, so this can never trivially "pass" without having
    // actually observed a real bus transaction (the same mistake
    // already caught once in tb_can_regfile.v).
    task wait_for_rx_nonempty;
        integer guard;
        begin
            guard = 0;
            bus_read(ADDR_STATUS);
            while ((rd_result[3] === 1'b1) && (guard < 3000)) begin
                bus_read(ADDR_STATUS);
                guard = guard + 1;
            end
            check(rd_result[3] === 1'b0, "rx_fifo_became_nonempty");
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0; can_rx = `CAN_RECESSIVE;
        addr = 32'd0; wdata = 32'd0; we = 1'b0; re = 1'b0; sel = 1'b0;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // ============================================================
        // Setup: enable the controller, LOOPBACK on (Section 2.5),
        // and a small, comfortable bit-timing config: BRP=0 (1 sysclk
        // per TQ), PROP_SEG=2, PHASE_SEG1=2, PHASE_SEG2=2, SJW=1.
        // ============================================================
        bus_write(ADDR_CTRL, 32'b011); // EN=1, LOOPBACK=1, RESET=0
        bus_read(ADDR_CTRL);
        check(rd_result === 32'b011, "setup_ctrl_readback");

        bus_write(ADDR_BTIME, {10'd0, 2'd1, 4'd2, 4'd2, 4'd2, 8'd0});
        bus_read(ADDR_BTIME);
        check(rd_result === {10'd0, 2'd1, 4'd2, 4'd2, 4'd2, 8'd0}, "setup_btime_readback");

        // ============================================================
        // Scenario A: full DLC=8 frame, standard ID, round-tripped
        // through TX push -> wire encode -> loopback -> wire decode
        // -> RX FIFO -> register reads. This is Section 2.5's
        // required self-test.
        // ============================================================
        bus_write(ADDR_TX_ID, 32'h0000_0245);
        bus_write(ADDR_TX_DATA0, 32'hDEAD_BEEF);
        bus_write(ADDR_TX_DATA1, 32'h1234_5678);
        bus_write(ADDR_TX_CTRL, {26'd0, 1'b0 /*ide*/, 1'b0 /*rtr*/, 4'd8 /*dlc*/});

        wait_for_rx_nonempty;

        bus_read(ADDR_IP);
        check(rd_result[`CAN_INT_RX_BIT] === 1'b1, "A_rx_ip_pending");
        check(rd_result[`CAN_INT_TX_BIT] === 1'b1, "A_tx_ip_pending");
        check(rd_result[`CAN_INT_ERR_BIT] === 1'b0, "A_err_ip_not_pending");

        bus_read(ADDR_RX_ID);
        check(rd_result === 32'h0000_0245, "A_rx_id");
        bus_read(ADDR_RX_DATA0);
        check(rd_result === 32'hDEAD_BEEF, "A_rx_data0");
        bus_read(ADDR_RX_DATA1);
        check(rd_result === 32'h1234_5678, "A_rx_data1");
        bus_read(ADDR_RX_CTRL); // pop strobe
        check(rd_result === {26'd0, 1'b0, 1'b0, 4'd8}, "A_rx_ctrl_ide_rtr_dlc");

        bus_read(ADDR_STATUS);
        check(rd_result[3] === 1'b1, "A_rx_fifo_empty_after_pop");
        check(rd_result[5:4] === `CAN_ERR_ACTIVE, "A_err_state_stayed_active");

        bus_read(ADDR_TEC_REC);
        check(rd_result === 32'd0, "A_tec_rec_clean");

        // Clear IP for the next scenario.
        bus_write(ADDR_IP, 32'b011);
        bus_read(ADDR_IP);
        check(rd_result[`CAN_INT_RX_BIT] === 1'b0, "A_rx_ip_cleared");
        check(rd_result[`CAN_INT_TX_BIT] === 1'b0, "A_tx_ip_cleared");

        // ============================================================
        // Scenario B: DLC=0 - no Data field at all. Confirms the
        // CONTROL->CRC skip (Step 5, already unit-tested) still
        // produces a correct, receivable frame once wired through
        // the full stack, and that a 0-length payload round-trips
        // as "no data bytes" rather than stale leftover content.
        // ============================================================
        bus_write(ADDR_TX_ID, 32'h0000_0001);
        bus_write(ADDR_TX_CTRL, {26'd0, 1'b0, 1'b0, 4'd0});

        wait_for_rx_nonempty;

        bus_read(ADDR_RX_ID);
        check(rd_result === 32'h0000_0001, "B_rx_id");
        bus_read(ADDR_RX_CTRL);
        check(rd_result === {26'd0, 1'b0, 1'b0, 4'd0}, "B_rx_ctrl_dlc0");

        bus_read(ADDR_TEC_REC);
        check(rd_result === 32'd0, "B_tec_rec_still_clean");

        // ============================================================
        // Scenario C: variable DLC=3 (partial data word) plus an
        // interrupt-masking check at the top-level irq_rx/irq_tx
        // PINS (not just the CAN_IP register bits, already covered
        // by tb_can_regfile.v) - only RX is enabled in CAN_IE, so
        // irq_rx must assert and irq_tx must not, even though both
        // CAN_IP bits get set underneath.
        // ============================================================
        bus_write(ADDR_IE, 32'b001); // RX only

        bus_write(ADDR_TX_ID, 32'h0000_0123);
        bus_write(ADDR_TX_DATA0, 32'h00AA_BBCC); // bytes 0/1/2 = CC/BB/AA
        bus_write(ADDR_TX_DATA1, 32'h0000_0000);
        bus_write(ADDR_TX_CTRL, {26'd0, 1'b0, 1'b0, 4'd3});

        wait_for_rx_nonempty;

        check(irq_rx === 1'b1, "C_irq_rx_pin_asserted");
        check(irq_tx === 1'b0, "C_irq_tx_pin_masked");

        bus_read(ADDR_RX_DATA0);
        check(rd_result === 32'h00AA_BBCC, "C_rx_data0_three_bytes");
        bus_read(ADDR_RX_DATA1);
        check(rd_result === 32'h0000_0000, "C_rx_data1_untouched");
        bus_read(ADDR_RX_CTRL);
        check(rd_result === {26'd0, 1'b0, 1'b0, 4'd3}, "C_rx_ctrl_dlc3");

        // Re-enable all interrupts and clear pending for a clean end state.
        bus_write(ADDR_IE, 32'b111);
        bus_write(ADDR_IP, 32'b111);

        repeat (20) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_controller_top: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_controller_top: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
