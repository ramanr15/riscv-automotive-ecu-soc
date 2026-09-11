`ifndef RV32_DEF_V
`define RV32_DEF_V

// ============================================================
// RV32I / RV32M OPCODES
// ============================================================

`define OPCODE_LOAD      7'b0000011
`define OPCODE_MISC_MEM  7'b0001111
`define OPCODE_OP_IMM    7'b0010011
`define OPCODE_AUIPC     7'b0010111
`define OPCODE_STORE     7'b0100011
`define OPCODE_OP        7'b0110011
`define OPCODE_LUI       7'b0110111
`define OPCODE_BRANCH    7'b1100011
`define OPCODE_JALR      7'b1100111
`define OPCODE_JAL       7'b1101111
`define OPCODE_SYSTEM    7'b1110011

// ============================================================
// ALU CONTROL
// ============================================================

`define ALU_ADD   4'd0
`define ALU_SUB   4'd1
`define ALU_AND   4'd2
`define ALU_OR    4'd3
`define ALU_XOR   4'd4
`define ALU_SLL   4'd5
`define ALU_SRL   4'd6
`define ALU_SRA   4'd7
`define ALU_SLT   4'd8
`define ALU_SLTU  4'd9

// ============================================================
// EXCEPTION CAUSES
// ============================================================

`define CAUSE_INST_MISALIGNED   32'd0
`define CAUSE_ILLEGAL_INST      32'd2
`define CAUSE_BREAKPOINT        32'd3
`define CAUSE_LOAD_MISALIGNED   32'd4
`define CAUSE_STORE_MISALIGNED  32'd6
`define CAUSE_ECALL_MMODE       32'd11

`endif