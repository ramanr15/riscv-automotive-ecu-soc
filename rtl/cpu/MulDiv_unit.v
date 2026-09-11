`timescale 1ns / 1ps
// ============================================================
// muldiv_unit  -- MULTI-CYCLE REWRITE
//
// WHY THIS CHANGED
//   The original version computed *, /, and % combinationally.
//   Vivado built the divider as a ~32-deep ripple chain of
//   subtract-and-compare stages: 293 CARRY4 levels, 90 ns of
//   delay, WNS -80 ns on a 10 ns clock. No amount of
//   optimisation fixes a single-cycle 32-bit divide; the fix
//   has to be structural.
//
// WHAT IT DOES NOW
//   All operations first spend one cycle capturing the raw
//   operands (see the operand-capture note below).
//
//   MUL / MULH / MULHSU / MULHU : 4 cycles
//       stage 1  register operands
//       stage 2  four 16x16 partial products  (one DSP each)
//       stage 3  64-bit partial-product sum
//       stage 4  sign fix-up and half select
//   DIV / DIVU / REM / REMU     : 35 cycles
//       restoring division, one 32-bit subtract per cycle
//   Divide by zero and the 0x80000000 / -1 overflow case
//       complete in 2 cycles, per the RISC-V spec.
//
//   Both signed and unsigned cases go through a single
//   unsigned datapath: operands are converted to magnitudes on
//   entry and the sign is reapplied on exit. That keeps one
//   multiplier and one divider instead of four of each.
//
// HANDSHAKE
//   req  : level, "EX currently holds an RV32M instruction"
//   busy : level, "stall EX" -- drops for exactly one cycle
//          when the result is valid, which is when the
//          pipeline advances and captures it.
//
//   done_flag makes the handshake self-clearing, so two
//   back-to-back RV32M instructions each get a fresh start.
// ============================================================
module muldiv_unit (
    input  wire        clk,
    input  wire        reset,

    input  wire        req,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b,
    input  wire [2:0]  funct3,

    output wire [31:0] result,
    output wire        busy
);

    localparam S_IDLE   = 3'd0;
    localparam S_LOAD   = 3'd1;
    localparam S_MUL2   = 3'd2;
    localparam S_MUL3   = 3'd3;
    localparam S_DIV    = 3'd4;
    localparam S_DIVFIN = 3'd5;

    reg [2:0]  state;
    reg        done_flag;
    reg [31:0] result_r;

    assign result = result_r;
    assign busy   = req & ~done_flag;

    // ------------------------------------------------------------
    // Operand capture
    //
    // The raw operands are registered on the cycle the request
    // arrives, and every use below reads the registered copies.
    //
    // This matters for timing. forwarded_a/b come off the end of
    // the forwarding mux, which is itself several levels deep and
    // physically far from the DSP column. Feeding that straight
    // into the magnitude negate put the forwarding chain, a
    // 32-bit two's-complement adder and a long route all in one
    // clock period -- a 7.6 ns path, WNS -1.5 ns. Splitting at a
    // register boundary costs one cycle and closes it.
    // ------------------------------------------------------------
    reg [31:0] a_r;
    reg [31:0] b_r;
    reg [2:0]  f3_r;

    //   000 MUL     001 MULH    010 MULHSU  011 MULHU
    //   100 DIV     101 DIVU    110 REM     111 REMU
    wire is_div     = f3_r[2];
    wire is_rem     = f3_r[2] & f3_r[1];
    wire div_signed = (f3_r == 3'b100) | (f3_r == 3'b110);

    wire mul_a_signed = (f3_r == 3'b000) |
                        (f3_r == 3'b001) |
                        (f3_r == 3'b010);

    wire mul_b_signed = (f3_r == 3'b000) |
                        (f3_r == 3'b001);

    wire take_high = (f3_r == 3'b001) |
                     (f3_r == 3'b010) |
                     (f3_r == 3'b011);

    // Sign of each operand, per operation
    wire na = is_div ? (div_signed   & a_r[31])
                     : (mul_a_signed & a_r[31]);

    wire nb = is_div ? (div_signed   & b_r[31])
                     : (mul_b_signed & b_r[31]);

    // Magnitudes
    wire [31:0] mag_a = na ? (~a_r + 32'd1) : a_r;
    wire [31:0] mag_b = nb ? (~b_r + 32'd1) : b_r;

    // ------------------------------------------------------------
    // Multiplier registers
    // ------------------------------------------------------------
    reg [31:0] pp0, pp1, pp2, pp3;
    reg [63:0] prod;
    reg        neg_prod;
    reg        take_high_r;

    wire [63:0] prod_final = neg_prod ? (~prod + 64'd1) : prod;

    // ------------------------------------------------------------
    // Divider registers
    //   sr = { remainder[31:0], quotient/dividend[31:0] }
    // ------------------------------------------------------------
    reg [63:0] sr;
    reg [31:0] divisor_r;
    reg [5:0]  cnt;
    reg        quo_neg;
    reg        rem_neg;
    reg        is_rem_r;

    // One restoring-division step: shift left, then conditionally
    // subtract the divisor from the upper half.
    wire [63:0] sr_shift = {sr[62:0], 1'b0};
    wire [32:0] sub_res  = {1'b0, sr_shift[63:32]} - {1'b0, divisor_r};
    wire        fits     = ~sub_res[32];

    wire [63:0] sr_next  = fits ? {sub_res[31:0], sr_shift[31:1], 1'b1}
                                : sr_shift;

    wire [31:0] quo_mag = sr[31:0];
    wire [31:0] rem_mag = sr[63:32];

    wire [31:0] div_out =
        is_rem_r ? (rem_neg ? (~rem_mag + 32'd1) : rem_mag)
                 : (quo_neg ? (~quo_mag + 32'd1) : quo_mag);

    // ------------------------------------------------------------
    // Control
    // ------------------------------------------------------------
    always @(posedge clk) begin

        if (reset) begin
            state       <= S_IDLE;
            done_flag   <= 1'b0;
            result_r    <= 32'b0;
            cnt         <= 6'd0;
            prod        <= 64'b0;
            sr          <= 64'b0;
            divisor_r   <= 32'b0;
            pp0         <= 32'b0;
            pp1         <= 32'b0;
            pp2         <= 32'b0;
            pp3         <= 32'b0;
            neg_prod    <= 1'b0;
            take_high_r <= 1'b0;
            quo_neg     <= 1'b0;
            rem_neg     <= 1'b0;
            is_rem_r    <= 1'b0;
            a_r         <= 32'b0;
            b_r         <= 32'b0;
            f3_r        <= 3'b0;
        end

        else begin

            case (state)

                // ----------------------------------------------
                S_IDLE: begin

                    // Result has been consumed by the pipeline;
                    // re-arm for the next RV32M instruction.
                    if (!busy)
                        done_flag <= 1'b0;

                    // Capture the operands and nothing else, so
                    // the forwarding mux ends at a flip-flop.
                    if (req && !done_flag) begin
                        a_r   <= operand_a;
                        b_r   <= operand_b;
                        f3_r  <= funct3;
                        state <= S_LOAD;
                    end
                end

                // ----------------------------------------------
                S_LOAD: begin

                    if (!is_div) begin
                        // ---- multiply: capture partials ----
                        pp0 <= mag_a[15:0]  * mag_b[15:0];
                        pp1 <= mag_a[15:0]  * mag_b[31:16];
                        pp2 <= mag_a[31:16] * mag_b[15:0];
                        pp3 <= mag_a[31:16] * mag_b[31:16];

                        neg_prod    <= na ^ nb;
                        take_high_r <= take_high;

                        state <= S_MUL2;
                    end

                    else if (b_r == 32'b0) begin
                        // ---- divide by zero ----
                        result_r  <= is_rem ? a_r : 32'hFFFFFFFF;
                        done_flag <= 1'b1;
                        state     <= S_IDLE;
                    end

                    else if (div_signed &&
                             (a_r == 32'h80000000) &&
                             (b_r == 32'hFFFFFFFF)) begin
                        // ---- signed overflow ----
                        result_r  <= is_rem ? 32'b0 : 32'h80000000;
                        done_flag <= 1'b1;
                        state     <= S_IDLE;
                    end

                    else begin
                        // ---- normal divide ----
                        sr        <= {32'b0, mag_a};
                        divisor_r <= mag_b;
                        quo_neg   <= na ^ nb;
                        rem_neg   <= na;
                        is_rem_r  <= is_rem;
                        cnt       <= 6'd32;
                        state     <= S_DIV;
                    end
                end

                // ----------------------------------------------
                S_MUL2: begin
                    prod <= {32'd0, pp0}
                          + {16'd0, pp1, 16'd0}
                          + {16'd0, pp2, 16'd0}
                          + {pp3, 32'd0};
                    state <= S_MUL3;
                end

                // ----------------------------------------------
                S_MUL3: begin
                    result_r  <= take_high_r ? prod_final[63:32]
                                             : prod_final[31:0];
                    done_flag <= 1'b1;
                    state     <= S_IDLE;
                end

                // ----------------------------------------------
                S_DIV: begin
                    sr  <= sr_next;
                    cnt <= cnt - 6'd1;
                    if (cnt == 6'd1)
                        state <= S_DIVFIN;
                end

                // ----------------------------------------------
                S_DIVFIN: begin
                    result_r  <= div_out;
                    done_flag <= 1'b1;
                    state     <= S_IDLE;
                end

                // ----------------------------------------------
                default: begin
                    state <= S_IDLE;
                end

            endcase

        end

    end

endmodule
