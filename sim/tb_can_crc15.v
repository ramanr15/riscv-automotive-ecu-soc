`timescale 1ns / 1ps

// ============================================================
// tb_can_crc15.v
//
// Step 4 self-check, per Section 2.3.11 item 2 / Section 4.4:
// "CRC-15 module, unit-tested against hand-computed CRC values
// from a known 8-byte frame" and "inject a single bit-flip into a
// transmitted frame and confirm the CRC checker flags it as
// corrupted."
//
// Expected CRC values below were computed with an independent
// Python reference model of the same Bosch CAN CRC-15 LFSR
// (docs/can_crc15_reference.py) - NOT copied from this RTL - so
// this is a genuine cross-check of the algorithm, not a
// self-fulfilling comparison.
//
// Vector 1: SOF(0) + ID=0x123 (11b) + RTR=0,IDE=0,r0=0 + DLC=8
//           + data bytes 0x00..0x07               -> CRC 0x234B
// Vector 2: all-dominant content, same length       -> CRC 0x0000
// Vector 3: SOF(0) + ID=0x000 + RTR=1,IDE=0,r0=0,DLC=0 -> CRC 0x73C5
// Vector 4: Vector 1 with bit[10] flipped            -> CRC 0x357D
//           (must differ from Vector 1's CRC)
// ============================================================

module tb_can_crc15;

    reg clk, rst_n;
    reg reset_crc, shift_en, data_bit;
    wire [14:0] crc_reg;

    integer errors;

    can_crc15 DUT (
        .clk(clk), .rst_n(rst_n),
        .reset_crc(reset_crc),
        .shift_en(shift_en),
        .data_bit(data_bit),
        .crc_reg(crc_reg)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_equal15;
        input [14:0] actual, expected;
        input [255:0] name;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s = 0x%04h, expected 0x%04h @ %0t",
                          name, actual, expected, $time);
                errors = errors + 1;
            end
            else begin
                $display("PASS: %0s = 0x%04h @ %0t", name, actual, $time);
            end
        end
    endtask

    // Feed `len` bits of `seq` (MSB of seq[len-1:0] first, matching
    // CAN's MSB-first bit order within a field) through the CRC.
    task run_vector;
        input [127:0] seq;
        input integer len;
        input [14:0] expected;
        input [255:0] name;
        integer k;
        begin
            @(negedge clk);
            reset_crc = 1'b1;
            @(negedge clk);
            reset_crc = 1'b0;

            for (k = len - 1; k >= 0; k = k - 1) begin
                @(negedge clk);
                data_bit = seq[k];
                shift_en = 1'b1;
                @(negedge clk);
                shift_en = 1'b0;
            end

            check_equal15(crc_reg, expected, name);
        end
    endtask

    reg [127:0] v1, v2, v3, v4;

    initial begin
        errors = 0;
        rst_n = 1'b0;
        reset_crc = 1'b0; shift_en = 1'b0; data_bit = 1'b0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        // ---- Vector 1: SOF + ID 0x123 + RTR/IDE/r0=0 + DLC=8 + 8 data bytes ----
        // Bit order (MSB first, 83 bits total):
        //   [0]      SOF = 0
        //   [1:11]   ID = 0x123 (11 bits)
        //   [12]     RTR = 0
        //   [13]     IDE = 0
        //   [14]     r0  = 0
        //   [15:18]  DLC = 8 (4 bits)
        //   [19:82]  data bytes 0x00,0x01,...,0x07 (64 bits, MSB first per byte)
        v1 = { 1'b0,
               11'h123,
               1'b0, 1'b0, 1'b0, 4'd8,
               8'h00, 8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07 };
        run_vector(v1, 83, 15'h234B, "vector1_known_8byte_frame");

        // ---- Vector 2: all-dominant content, same length ----
        v2 = 128'd0;
        run_vector(v2, 83, 15'h0000, "vector2_all_dominant");

        // ---- Vector 3: SOF + ID 0x000 + RTR=1,IDE=0,r0=0,DLC=0 (19 bits) ----
        v3 = { 1'b0, 11'h000, 1'b1, 1'b0, 1'b0, 4'd0 };
        run_vector(v3, 19, 15'h73C5, "vector3_rtr_dlc0");

        // ---- Vector 4: vector1 with bit index 10 (0-based from MSB) flipped ----
        // Original v1 bit[10] (counting MSB=bit82 downward, i.e. the
        // same indexing run_vector uses: seq[k] for k=len-1..0) sits
        // inside the ID field. We flip the same logical position the
        // Python reference model flipped (list index 10, 0-based from
        // the start of the sequence).
        v4 = v1;
        v4[83 - 1 - 10] = ~v4[83 - 1 - 10];
        run_vector(v4, 83, 15'h357D, "vector4_single_bit_corruption");

        if (crc_reg === 15'h234B)
            $display("NOTE: corrupted-frame CRC unexpectedly matched the clean CRC!");

        repeat (5) @(posedge clk);

        if (errors == 0)
            $display("\n=== tb_can_crc15: ALL CHECKS PASSED ===\n");
        else
            $display("\n=== tb_can_crc15: %0d CHECK(S) FAILED ===\n", errors);

        $finish;
    end

endmodule
