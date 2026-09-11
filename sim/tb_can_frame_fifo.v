`timescale 1ns / 1ps

// ============================================================
// tb_can_frame_fifo.v
//
// Step 2 self-check, per Section 2.3.11 item 3: "TX/RX FIFOs,
// unit-tested with push/pop sequences."
//
// Covers: fill to full, drain to empty, full/empty flag
// correctness, FIFO ordering (first pushed == first popped), and
// the empty->nonempty / full->nonfull interrupt pulses.
// ============================================================

module tb_can_frame_fifo;

    localparam DEPTH = 8;

    reg clk, rst_n;
    reg push, pop;
    reg [28:0] id_in;
    reg        ide_in, rtr_in;
    reg [3:0]  dlc_in;
    reg [63:0] data_in;

    wire [28:0] id_out;
    wire        ide_out, rtr_out;
    wire [3:0]  dlc_out;
    wire [63:0] data_out;
    wire full, empty;
    wire empty_to_nonempty, full_to_nonfull;

    integer errors;

    can_frame_fifo #(.DEPTH(DEPTH)) DUT (
        .clk(clk), .rst_n(rst_n),
        .push(push), .id_in(id_in), .ide_in(ide_in), .rtr_in(rtr_in),
        .dlc_in(dlc_in), .data_in(data_in),
        .pop(pop), .id_out(id_out), .ide_out(ide_out), .rtr_out(rtr_out),
        .dlc_out(dlc_out), .data_out(data_out),
        .full(full), .empty(empty),
        .empty_to_nonempty(empty_to_nonempty),
        .full_to_nonfull(full_to_nonfull)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

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

    task do_push;
        input [28:0] id;
        input [63:0] data;
        begin
            @(negedge clk);
            push    = 1'b1;
            id_in   = id;
            ide_in  = 1'b0;
            rtr_in  = 1'b0;
            dlc_in  = 4'd8;
            data_in = data;
            @(negedge clk);
            push = 1'b0;
        end
    endtask

    task do_pop;
        begin
            @(negedge clk);
            pop = 1'b1;
            @(negedge clk);
            pop = 1'b0;
        end
    endtask

    integer i;

    initial begin
        errors = 0;
        rst_n  = 1'b0;
        push   = 1'b0;
        pop    = 1'b0;
        id_in  = 0; ide_in = 0; rtr_in = 0; dlc_in = 0; data_in = 0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        check(empty, "reset_state_is_empty");
        check(!full, "reset_state_not_full");

        // ---- Fill to exactly full with DEPTH frames ----
        for (i = 0; i < DEPTH; i = i + 1) begin
            check(!full, "not_full_before_pushN");
            do_push(29'h100 + i, {32'hCAFEBABE, 32'h0 + i});
        end
        check(full, "full_after_DEPTH_pushes");
        check(!empty, "not_empty_after_pushes");

        // A push while full must be silently dropped (no corruption)
        do_push(29'h1FF, 64'hDEAD_DEAD_DEAD_DEAD);
        check(full, "still_full_after_overflow_push_attempt");

        // ---- Drain and check FIFO ordering (first in == first out) ----
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(negedge clk); // let combinational head settle
            check(id_out === (29'h100 + i), "fifo_order_id");
            check(data_out[31:0] === (32'h0 + i), "fifo_order_data");
            do_pop;
        end
        check(empty, "empty_after_full_drain");

        // A pop while empty must not corrupt state
        do_pop;
        check(empty, "still_empty_after_underflow_pop_attempt");

        // ---- empty_to_nonempty pulse on first push into empty FIFO ----
        @(negedge clk);
        push = 1'b1; id_in = 29'h55; ide_in = 0; rtr_in = 0; dlc_in = 4'd2; data_in = 64'hABCD;
        @(posedge clk); #1;
        // pulse appears one clk after the push edge is registered
        @(posedge clk); #1;
        check(empty_to_nonempty === 1'b1, "empty_to_nonempty_pulses");
        @(negedge clk);
        push = 1'b0;

        // ---- full_to_nonfull pulse when draining a full FIFO ----
        for (i = 0; i < DEPTH - 1; i = i + 1)
            do_push(29'h200 + i, {32'h0, 32'h0});
        check(full, "refilled_to_full");
        do_pop;
        @(posedge clk); #1;
        check(full_to_nonfull === 1'b1, "full_to_nonfull_pulses_on_first_pop");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_frame_fifo: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_frame_fifo: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
