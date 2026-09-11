`timescale 1ns / 1ps

// ============================================================
// tb_can_stuffing.v
//
// Step 3 self-check, per Section 2.3.11 item 5: "Bit
// stuffing/destuffing, tested with an all-ones and an all-zeros
// payload to force stuffing to trigger."
//
// Flow: a raw test bit sequence is fed, one bit per bit_tick,
// into can_stuff_tx. Its tx_bit output (the "wire" bitstream,
// including inserted stuff bits) is captured and then replayed,
// one bit per bit_tick, into can_destuff_rx. The bits it marks
// valid_bit are compared back against the original raw sequence.
// A final test corrupts one stuff bit's polarity and confirms
// stuff_error fires.
// ============================================================

module tb_can_stuffing;

    reg clk, rst_n;

    // ---- TX stuffer ----
    reg  stuff_enable_tx, reset_stuff_tx, bit_tick_tx, raw_bit;
    wire tx_bit, stuff_inserted;
    wire [2:0] same_count_tx;

    can_stuff_tx TX (
        .clk(clk), .rst_n(rst_n),
        .stuff_enable(stuff_enable_tx),
        .reset_stuff(reset_stuff_tx),
        .bit_tick(bit_tick_tx),
        .raw_bit(raw_bit),
        .tx_bit(tx_bit),
        .stuff_inserted(stuff_inserted),
        .same_count(same_count_tx)
    );

    // ---- RX destuffer ----
    reg  stuff_enable_rx, reset_stuff_rx, bit_tick_rx, rx_bit;
    wire valid_bit, stuff_error;
    wire [2:0] same_count_rx;

    can_destuff_rx RX (
        .clk(clk), .rst_n(rst_n),
        .stuff_enable(stuff_enable_rx),
        .reset_stuff(reset_stuff_rx),
        .bit_tick(bit_tick_rx),
        .rx_bit(rx_bit),
        .valid_bit(valid_bit),
        .same_count(same_count_rx),
        .stuff_error(stuff_error)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;
    reg stuff_error_seen;

    always @(posedge clk) begin
        if (stuff_error) stuff_error_seen <= 1'b1;
    end

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

    // Wide enough scratch storage for a payload plus worst-case
    // stuff-bit overhead (roughly +1 stuffed bit per 4 raw bits).
    reg [127:0] wire_bits;
    integer     wire_len;

    reg [127:0] recovered_bits;
    integer     recovered_len;

    integer i2;

    task do_bit_tick_tx;
        begin
            @(negedge clk);
            bit_tick_tx = 1'b1;
            @(posedge clk); #1;
            wire_bits[wire_len] = tx_bit;
            wire_len = wire_len + 1;
            @(negedge clk);
            bit_tick_tx = 1'b0;
        end
    endtask

    task do_bit_tick_rx;
        begin
            @(negedge clk);
            bit_tick_rx = 1'b1;
            @(posedge clk); #1;
            if (valid_bit) begin
                recovered_bits[recovered_len] = rx_bit;
                recovered_len = recovered_len + 1;
            end
            @(negedge clk);
            bit_tick_rx = 1'b0;
        end
    endtask

    // Feed `len` bits of `pattern` (LSB first) through the TX
    // stuffer, then replay the captured wire bits through the RX
    // destuffer, then compare recovered_bits against `pattern`.
    task run_case;
        input [63:0] pattern;
        input integer len;
        input [255:0] name;
        integer k;
        reg pass;
        begin
            wire_len      = 0;
            recovered_len = 0;

            // ---- SOF: dominant bit, primes both stuffers ----
            @(negedge clk);
            reset_stuff_tx = 1'b1; reset_stuff_rx = 1'b1;
            @(negedge clk);
            reset_stuff_tx = 1'b0; reset_stuff_rx = 1'b0;

            stuff_enable_tx = 1'b1;
            stuff_enable_rx = 1'b1;

            raw_bit = 1'b0; // SOF is dominant
            do_bit_tick_tx;

            // NOTE: can_stuff_tx does NOT consume raw_bit on a tick
            // where it forces a stuff bit (stuff_inserted goes high
            // that same tick, since it's a registered output updated
            // on the same posedge do_bit_tick_tx samples). So only
            // advance to the next pattern bit when this tick's bit
            // was actually consumed - otherwise re-present the same
            // raw_bit next tick.
            k = 0;
            while (k < len) begin
                raw_bit = pattern[k];
                do_bit_tick_tx;
                if (!stuff_inserted) k = k + 1;
            end

            // Replay captured wire bitstream (SOF + payload,
            // including any inserted stuff bits) into the destuffer.
            for (k = 0; k < wire_len; k = k + 1) begin
                rx_bit = wire_bits[k];
                do_bit_tick_rx;
            end

            // recovered_bits[0] is the SOF bit (dominant); compare
            // recovered_bits[1 +: len] against the original pattern.
            pass = (recovered_bits[0] === 1'b0);
            for (k = 0; k < len; k = k + 1)
                if (recovered_bits[1 + k] !== pattern[k]) pass = 1'b0;

            check(pass, name);
            check(recovered_len == len + 1, {name, "_recovered_len"});
            $display("    wire_len=%0d recovered_len=%0d (expected %0d)",
                       wire_len, recovered_len, len + 1);
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0;
        stuff_enable_tx = 1'b0; reset_stuff_tx = 1'b0; bit_tick_tx = 1'b0; raw_bit = 1'b1;
        stuff_enable_rx = 1'b0; reset_stuff_rx = 1'b0; bit_tick_rx = 1'b0; rx_bit  = 1'b1;
        wire_bits = 0; recovered_bits = 0; wire_len = 0; recovered_len = 0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        // All-zeros payload (16 bits of 0, i.e. dominant): should
        // force stuff bits every 5 bits after the SOF dominant run.
        run_case(64'h0000_0000_0000_0000, 16, "all_zeros_payload_roundtrip");

        // All-ones payload (16 bits of 1, i.e. recessive): SOF is
        // dominant so the first 5-in-a-row run of recessive bits
        // starts one bit into the payload.
        run_case(64'hFFFF_FFFF_FFFF_FFFF, 16, "all_ones_payload_roundtrip");

        // Mixed payload with no long runs: expect wire_len == len+1
        // (no stuff bits inserted at all).
        run_case(64'h0000_0000_0000_AAAA, 16, "alternating_payload_no_stuffing");
        check(wire_len == 17, "alternating_payload_wire_len_unchanged");

        // ---- Stuff-error injection ----
        // Build a 6-dominant-bit run manually and corrupt the
        // expected stuff bit's polarity before replaying into RX.
        wire_len = 0; recovered_len = 0;
        @(negedge clk);
        reset_stuff_tx = 1'b1; reset_stuff_rx = 1'b1;
        @(negedge clk);
        reset_stuff_tx = 1'b0; reset_stuff_rx = 1'b0;
        stuff_enable_tx = 1'b1; stuff_enable_rx = 1'b1;

        raw_bit = 1'b0; do_bit_tick_tx; // SOF (dominant, run=1)
        raw_bit = 1'b0; do_bit_tick_tx; // run=2
        raw_bit = 1'b0; do_bit_tick_tx; // run=3
        raw_bit = 1'b0; do_bit_tick_tx; // run=4
        raw_bit = 1'b0; do_bit_tick_tx; // run=5 -> next tick auto-stuffs recessive
        raw_bit = 1'b0; do_bit_tick_tx; // stuff bit (should be recessive=1), run resets to 1

        // Corrupt the captured stuff bit (index 5) to dominant (0)
        // instead of the correct recessive (1) before replay.
        check(wire_bits[5] === 1'b1, "captured_stuff_bit_is_recessive_before_corruption");
        wire_bits[5] = 1'b0;

        stuff_error_seen = 1'b0;
        for (i2 = 0; i2 < wire_len; i2 = i2 + 1) begin
            rx_bit = wire_bits[i2];
            do_bit_tick_rx;
        end
        @(posedge clk); #1;
        check(stuff_error_seen === 1'b1, "corrupted_stuff_bit_raises_stuff_error");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_stuffing: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_stuffing: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
