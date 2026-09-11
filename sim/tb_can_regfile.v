`timescale 1ns / 1ps
`include "can_defs.v"

// ============================================================
// tb_can_regfile.v
//
// Step 9 (register half) self-check, per Person 3's own
// verification plan: "confirm every register in Person 1's
// register map can be written/read correctly and that
// write-1-to-clear semantics on CAN_IP behave as documented."
//
// Drives the bus side directly (no FIFOs/protocol stack involved -
// that integration is can_controller_top.v's job) and checks every
// register's write/read/strobe/masking behavior in isolation.
// ============================================================

module tb_can_regfile;

    reg clk, rst_n;
    reg  [31:0] addr, wdata;
    wire [31:0] rdata;
    reg  we, re, sel;
    wire ready;

    wire can_en, can_loopback, soft_reset;
    wire [7:0] btime_brp;
    wire [3:0] btime_prop_seg, btime_phase_seg1, btime_phase_seg2;
    wire [1:0] btime_sjw;

    reg tx_fifo_full, tx_fifo_empty, rx_fifo_full, rx_fifo_empty;
    reg [1:0] err_state;

    wire tx_push;
    wire [28:0] tx_push_id;
    wire tx_push_ide, tx_push_rtr;
    wire [3:0] tx_push_dlc;
    wire [63:0] tx_push_data;

    wire rx_pop;
    reg [28:0] rx_id;
    reg rx_ide, rx_rtr;
    reg [3:0] rx_dlc;
    reg [63:0] rx_data;

    reg rx_ip_set, tx_ip_set, err_ip_set;
    wire irq_rx, irq_tx, irq_err;

    reg [7:0] tec, rec;

    can_regfile DUT (
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
        .rx_pop(rx_pop), .rx_id(rx_id), .rx_ide(rx_ide), .rx_rtr(rx_rtr),
        .rx_dlc(rx_dlc), .rx_data(rx_data),
        .rx_ip_set(rx_ip_set), .tx_ip_set(tx_ip_set), .err_ip_set(err_ip_set),
        .irq_rx(irq_rx), .irq_tx(irq_tx), .irq_err(irq_err),
        .tec(tec), .rec(rec)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;
    reg [31:0] rd_result;

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

    initial begin
        errors = 0;
        rst_n = 1'b0;
        addr = 32'd0; wdata = 32'd0; we = 1'b0; re = 1'b0; sel = 1'b0;
        tx_fifo_full = 1'b0; tx_fifo_empty = 1'b1;
        rx_fifo_full = 1'b0; rx_fifo_empty = 1'b1;
        err_state = `CAN_ERR_ACTIVE;
        rx_id = 29'd0; rx_ide = 1'b0; rx_rtr = 1'b0; rx_dlc = 4'd0; rx_data = 64'd0;
        rx_ip_set = 1'b0; tx_ip_set = 1'b0; err_ip_set = 1'b0;
        tec = 8'd0; rec = 8'd0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        // ============================================================
        // Scenario A: CAN_CTRL - EN/LOOPBACK stick, RESET (bit 2)
        // pulses soft_reset once and never reads back as 1.
        // ============================================================
        check(ready === 1'b0, "reset_ready_low_when_not_selected");

        bus_write(32'h00, 32'b111); // EN=1, LOOPBACK=1, RESET=1
        check(can_en === 1'b1, "A_can_en_set");
        check(can_loopback === 1'b1, "A_can_loopback_set");
        check(soft_reset === 1'b1, "A_soft_reset_pulsed_on_write");

        bus_read(32'h00);
        check(rd_result === 32'b011, "A_ctrl_readback_reset_bit_always_0");

        // soft_reset must not still be asserted once we're not
        // writing CAN_CTRL any more.
        check(soft_reset === 1'b0, "A_soft_reset_deasserts_after_write");

        // ============================================================
        // Scenario B: CAN_BTIME field slicing.
        // BRP=8'hA5, PROP_SEG=4'h3, PHASE_SEG1=4'h5, PHASE_SEG2=4'h7,
        // SJW=2'h2 -> word = {10'd0,SJW,PHASE_SEG2,PHASE_SEG1,PROP_SEG,BRP}
        // ============================================================
        bus_write(32'h04, 32'h0027_53A5);
        check(btime_brp        === 8'hA5, "B_brp");
        check(btime_prop_seg   === 4'h3,  "B_prop_seg");
        check(btime_phase_seg1 === 4'h5,  "B_phase_seg1");
        check(btime_phase_seg2 === 4'h7,  "B_phase_seg2");
        check(btime_sjw        === 2'h2,  "B_sjw");

        bus_read(32'h04);
        check(rd_result === 32'h0027_53A5, "B_btime_readback");

        // ============================================================
        // Scenario C: CAN_STATUS reflects live status inputs.
        // ============================================================
        tx_fifo_full = 1'b1; tx_fifo_empty = 1'b0;
        rx_fifo_full = 1'b0; rx_fifo_empty = 1'b0;
        err_state = `CAN_ERR_PASSIVE;
        bus_read(32'h08);
        check(rd_result === {26'd0, `CAN_ERR_PASSIVE, 1'b0, 1'b0, 1'b0, 1'b1},
              "C_status_reflects_inputs");
        tx_fifo_full = 1'b0; tx_fifo_empty = 1'b1;
        rx_fifo_full = 1'b0; rx_fifo_empty = 1'b1;
        err_state = `CAN_ERR_ACTIVE;

        // ============================================================
        // Scenario D: TX push - ID/DATA0/DATA1 latch on their own
        // writes, CAN_TX_CTRL write is the same-cycle push strobe.
        // W-only registers read back as 0.
        // ============================================================
        bus_write(32'h0C, {3'd0, 29'h1234_5678} & 32'h1FFF_FFFF); // CAN_TX_ID
        bus_write(32'h10, 32'hAABB_CCDD);                          // CAN_TX_DATA0
        bus_write(32'h14, 32'h1122_3344);                          // CAN_TX_DATA1

        @(negedge clk);
        addr = 32'h18; wdata = {26'd0, 1'b1/*ide*/, 1'b0/*rtr*/, 4'd5/*dlc*/};
        we = 1'b1; re = 1'b0; sel = 1'b1;
        @(posedge clk); #1;
        check(tx_push === 1'b1, "D_tx_push_pulses_on_ctrl_write");
        check(tx_push_id === 29'h1234_5678, "D_tx_push_id");
        check(tx_push_dlc === 4'd5, "D_tx_push_dlc");
        check(tx_push_rtr === 1'b0, "D_tx_push_rtr");
        check(tx_push_ide === 1'b1, "D_tx_push_ide");
        check(tx_push_data === {32'h1122_3344, 32'hAABB_CCDD}, "D_tx_push_data");
        @(negedge clk);
        we = 1'b0; sel = 1'b0;
        #1; // let the combinational tx_push settle before reading it

        check(tx_push === 1'b0, "D_tx_push_is_one_shot");

        bus_read(32'h0C); check(rd_result === 32'd0, "D_tx_id_reads_zero");
        bus_read(32'h10); check(rd_result === 32'd0, "D_tx_data0_reads_zero");
        bus_read(32'h14); check(rd_result === 32'd0, "D_tx_data1_reads_zero");
        bus_read(32'h18); check(rd_result === 32'd0, "D_tx_ctrl_reads_zero");

        // ============================================================
        // Scenario E: RX pop - CAN_RX_ID/DATA0/DATA1 expose the
        // FIFO head continuously; reading CAN_RX_CTRL is the pop
        // strobe, and must pulse rx_pop on that exact read.
        // ============================================================
        rx_id = 29'h0765_4321; rx_ide = 1'b0; rx_rtr = 1'b1; rx_dlc = 4'd3;
        rx_data = 64'hDEAD_BEEF_0BAD_F00D;

        // Checked mid-transaction (re/sel still asserted, addr still
        // pointing at the register in question) - a post-hoc check
        // after the task deasserts re/sel would trivially read 0
        // regardless of correctness, since rx_pop is combinational.
        @(negedge clk);
        addr = 32'h1C; we = 1'b0; re = 1'b1; sel = 1'b1;
        #1;
        check(rdata === {3'd0, rx_id}, "E_rx_id_readback");
        check(rx_pop === 1'b0, "E_reading_rx_id_does_not_pop");
        @(posedge clk); @(negedge clk); re = 1'b0; sel = 1'b0;

        @(negedge clk);
        addr = 32'h20; we = 1'b0; re = 1'b1; sel = 1'b1;
        #1;
        check(rdata === 32'h0BAD_F00D, "E_rx_data0_readback");
        check(rx_pop === 1'b0, "E_reading_rx_data0_does_not_pop");
        @(posedge clk); @(negedge clk); re = 1'b0; sel = 1'b0;

        @(negedge clk);
        addr = 32'h24; we = 1'b0; re = 1'b1; sel = 1'b1;
        #1;
        check(rdata === 32'hDEAD_BEEF, "E_rx_data1_readback");
        check(rx_pop === 1'b0, "E_reading_rx_data1_does_not_pop");
        @(posedge clk); @(negedge clk); re = 1'b0; sel = 1'b0;

        @(negedge clk);
        addr = 32'h28; we = 1'b0; re = 1'b1; sel = 1'b1;
        #1;
        check(rx_pop === 1'b1, "E_rx_pop_pulses_on_rx_ctrl_read");
        check(rdata === {26'd0, rx_ide, rx_rtr, rx_dlc}, "E_rx_ctrl_readback");
        @(posedge clk);
        @(negedge clk);
        re = 1'b0; sel = 1'b0;
        #1; // let the combinational rx_pop settle before reading it
        check(rx_pop === 1'b0, "E_rx_pop_is_one_shot");

        // ============================================================
        // Scenario F: CAN_IE / CAN_IP - masking, write-1-to-clear
        // (only the written bit clears), and set-wins-over-clear on
        // a same-cycle collision.
        // ============================================================
        // F1: with IE all disabled, an event still sets IP but the
        // masked IRQ line stays low.
        bus_write(32'h2C, 32'd0); // CAN_IE = 0 (all masked)

        @(negedge clk);
        rx_ip_set = 1'b1;
        @(posedge clk); #1;
        @(negedge clk);
        rx_ip_set = 1'b0;

        bus_read(32'h30);
        check(rd_result[`CAN_INT_RX_BIT] === 1'b1, "F1_ip_rx_sets_even_when_masked");
        check(irq_rx === 1'b0, "F1_irq_rx_stays_low_when_masked");

        // F2: enabling IE unmasks the already-pending bit.
        bus_write(32'h2C, 32'b111); // CAN_IE = all enabled
        check(irq_rx === 1'b1, "F2_irq_rx_asserts_once_unmasked");

        // F3: write-1-to-clear only clears the written bit.
        @(negedge clk);
        tx_ip_set = 1'b1;
        @(posedge clk); #1;
        @(negedge clk);
        tx_ip_set = 1'b0;

        bus_read(32'h30);
        check(rd_result[`CAN_INT_RX_BIT] === 1'b1, "F3_ip_rx_still_set_before_clear");
        check(rd_result[`CAN_INT_TX_BIT] === 1'b1, "F3_ip_tx_now_set");

        bus_write(32'h30, (1 << `CAN_INT_RX_BIT)); // clear RX only

        bus_read(32'h30);
        check(rd_result[`CAN_INT_RX_BIT] === 1'b0, "F3_ip_rx_cleared");
        check(rd_result[`CAN_INT_TX_BIT] === 1'b1, "F3_ip_tx_untouched_by_rx_clear");
        check(irq_rx === 1'b0, "F3_irq_rx_deasserts_after_clear");
        check(irq_tx === 1'b1, "F3_irq_tx_still_asserted");

        // F4: same-cycle set-vs-clear on the same bit - the new
        // event must win, not the clear.
        @(negedge clk);
        addr = 32'h30; wdata = (1 << `CAN_INT_TX_BIT); // clear TX
        we = 1'b1; re = 1'b0; sel = 1'b1;
        tx_ip_set = 1'b1; // ...but a NEW tx event lands the same edge
        @(posedge clk); #1;
        @(negedge clk);
        we = 1'b0; sel = 1'b0; tx_ip_set = 1'b0;

        bus_read(32'h30);
        check(rd_result[`CAN_INT_TX_BIT] === 1'b1, "F4_same_cycle_set_wins_over_clear");

        // Clean up TX and check ERR path once more for completeness.
        bus_write(32'h30, (1 << `CAN_INT_TX_BIT));

        @(negedge clk);
        err_ip_set = 1'b1;
        @(posedge clk); #1;
        @(negedge clk);
        err_ip_set = 1'b0;
        check(irq_err === 1'b1, "F5_irq_err_asserts");

        // ============================================================
        // Scenario G: CAN_TEC_REC readback.
        // ============================================================
        tec = 8'd200; rec = 8'd50;
        bus_read(32'h34);
        check(rd_result === {16'd0, rec, tec}, "G_tec_rec_readback");

        // ============================================================
        // Scenario H: ready mirrors sel.
        // ============================================================
        @(negedge clk);
        sel = 1'b1; we = 1'b0; re = 1'b0; addr = 32'h00;
        #1;
        check(ready === 1'b1, "H_ready_follows_sel_high");
        @(negedge clk);
        sel = 1'b0;
        #1;
        check(ready === 1'b0, "H_ready_follows_sel_low");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_regfile: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_regfile: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
