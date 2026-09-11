`timescale 1ns / 1ps
`include "Def.v"

// ============================================================
// top_level_riscv  -- MODIFIED BY PERSON 2
//
// CHANGES vs the original CPU-only version:
//   1. data_mem is no longer instantiated here. The MEM stage
//      now drives an external memory port with exactly the same
//      timing (request in cycle N, read data in cycle N+1), and
//      periph_subsystem provides both the data memory and every
//      peripheral behind an address decoder.
//   2. irq_external input, wired into csr_unit.
//   3. pipeline_valid_ex flag, so an interrupt is only taken on
//      a real instruction and mepc never points at a bubble.
//   4. trap_ex = exception_ex | irq_take replaces exception_ex
//      everywhere a trap must redirect or squash.
// Everything else is untouched.
// ============================================================

module top_level_riscv (
    input  wire        clk,
    input  wire        reset,

    // ---------------- external interrupt ----------------
    input  wire        irq_external,

    // ---------------- data memory / peripheral port ----------------
    output wire [31:0] dmem_addr,
    output wire [31:0] dmem_wdata,
    output wire [3:0]  dmem_wstrb,
    output wire        dmem_we,
    output wire        dmem_re,
    input  wire [31:0] dmem_rdata,

    output wire [31:0] debug_pc,
    output wire [31:0] debug_instruction,
    output wire [31:0] debug_alu_result,
    output wire [31:0] debug_wb_data,
    output wire [4:0]  debug_wb_rd,
    output wire        debug_reg_write,
    output wire        debug_stall,
    output wire        debug_branch_taken,
    output wire        debug_irq_taken,
    output wire        debug_muldiv_busy
);

    // ============================================================
    // IF STAGE
    // ============================================================

    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] pc_plus4_if;
    wire [31:0] instruction_if;

    assign pc_plus4_if = pc + 32'd4;

    // ============================================================
    // IF DECODING FOR PREDICTION
    // ============================================================

    wire [6:0] opcode_if;
    wire [31:0] immediate_if;

    assign opcode_if = instruction_if[6:0];

    imm_gen IMM_IF (
        .instruction(instruction_if),
        .immediate(immediate_if)
    );

    wire is_branch_if;
    wire predicted_taken_raw_if;
    wire predicted_taken_if;
    wire [31:0] predicted_target_if;

    assign is_branch_if =
        (opcode_if == `OPCODE_BRANCH);

    assign predicted_taken_if =
        is_branch_if && predicted_taken_raw_if;

    assign predicted_target_if =
        pc + immediate_if;

    // ============================================================
    // PROGRAM COUNTER
    // ============================================================

wire stall;
wire redirect_ex;
wire exception_ex;
wire mret_ex;
wire fence_i_ex;
wire pipeline_flush;

// NEW: interrupt handling
wire interrupt_request;
wire irq_take;
wire trap_ex;

// NEW: multi-cycle RV32M stall.
// muldiv_unit now takes several cycles, so EX must be held
// rather than bubbled: the instruction stays put until its
// result is ready.
wire muldiv_busy;
wire ex_stall;
assign ex_stall = muldiv_busy;

// ID/EX pipeline PC
reg [31:0] pc_ex;

// NEW: does EX hold a real instruction (not a bubble)?
reg pipeline_valid_ex;

wire [31:0] recovery_pc_ex;

wire [31:0] mtvec;
wire [31:0] mepc;

    // Take an external interrupt only on a real EX instruction,
    // and never on the same cycle as a synchronous exception.
    // Never trap out of a half-finished multiply or divide:
    // the EX instruction is held, so it would restart anyway,
    // and the muldiv FSM would be left mid-sequence.
    assign irq_take =
        interrupt_request &&
        pipeline_valid_ex &&
        !exception_ex &&
        !ex_stall;

    assign trap_ex = exception_ex | irq_take;

    assign pipeline_flush =
        trap_ex      |
        mret_ex      |
        redirect_ex  |
        fence_i_ex;

    assign next_pc =
        trap_ex      ? mtvec :
        mret_ex      ? mepc :
        redirect_ex  ? recovery_pc_ex :
        fence_i_ex   ? pc_plus4_if :
        predicted_taken_if ? predicted_target_if :
        pc_plus4_if;

    prog_counter PROGRAM_COUNTER (
        .clk(clk),
        .reset(reset),
        .stall((stall || ex_stall) && !pipeline_flush),
        .next_pc(next_pc),
        .pc(pc)
    );

    // ============================================================
    // INSTRUCTION MEMORY
    // ============================================================

    instr_mem INSTRUCTION_MEMORY (
        .addr(pc),
        .instruction(instruction_if)
    );

    // ============================================================
    // BRANCH PREDICTOR
    // ============================================================

    wire branch_ex;
    wire actual_branch_taken_ex;

    branch_predictor BRANCH_PREDICTOR (
        .clk(clk),
        .reset(reset),

        .fetch_pc(pc),

        .update_enable(branch_ex),
        .update_pc(pc_ex),
        .actual_taken(actual_branch_taken_ex),

        .predicted_taken(predicted_taken_raw_if)
    );

    // ============================================================
    // IF / ID PIPELINE REGISTER
    // ============================================================

    reg [31:0] pc_id;
    reg [31:0] instruction_id;

    reg predicted_taken_id;
    reg [31:0] predicted_target_id;

    reg valid_id;

    always @(posedge clk) begin

        if (reset) begin

            pc_id <= 32'b0;
            instruction_id <= 32'h00000013;

            predicted_taken_id <= 1'b0;
            predicted_target_id <= 32'b0;

            valid_id <= 1'b0;

        end

        else if (ex_stall) begin

            // Hold: EX is busy with a multi-cycle operation.

        end

        else if (pipeline_flush) begin

            pc_id <= 32'b0;
            instruction_id <= 32'h00000013;

            predicted_taken_id <= 1'b0;
            predicted_target_id <= 32'b0;

            valid_id <= 1'b0;

        end

        else if (!stall) begin

            pc_id <= pc;
            instruction_id <= instruction_if;

            predicted_taken_id <= predicted_taken_if;
            predicted_target_id <= predicted_target_if;

            valid_id <= 1'b1;

        end

    end

    // ============================================================
    // ID STAGE
    // ============================================================

    wire [6:0] opcode_id;
    wire [4:0] rd_id;
    wire [2:0] funct3_id;
    wire [4:0] rs1_id;
    wire [4:0] rs2_id;
    wire [6:0] funct7_id;

    decoder DECODER (
        .instruction(instruction_id),

        .opcode(opcode_id),
        .rd(rd_id),
        .funct3(funct3_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .funct7(funct7_id)
    );

    wire [31:0] immediate_id;

    imm_gen IMMEDIATE_GENERATOR (
        .instruction(instruction_id),
        .immediate(immediate_id)
    );

    // ============================================================
    // CONTROL UNIT
    // ============================================================

    wire reg_write_id;
    wire mem_read_id;
    wire mem_write_id;
    wire mem_to_reg_id;
    wire alu_src_id;

    wire branch_id;
    wire jump_id;
    wire jalr_id;

    wire lui_id;
    wire auipc_id;

    wire system_id;
    wire csr_id;

    wire fence_id;
    wire fence_i_id;

    wire valid_major_id;

    control_unit CONTROL_UNIT (
        .opcode(opcode_id),
        .funct3(funct3_id),

        .reg_write(reg_write_id),

        .mem_read(mem_read_id),
        .mem_write(mem_write_id),

        .mem_to_reg(mem_to_reg_id),

        .alu_src(alu_src_id),

        .branch(branch_id),

        .jump(jump_id),
        .jalr(jalr_id),

        .lui(lui_id),
        .auipc(auipc_id),

        .system(system_id),
        .csr(csr_id),

        .fence(fence_id),
        .fence_i(fence_i_id),

        .valid_instruction(valid_major_id)
    );

    // ============================================================
    // ADDITIONAL INSTRUCTION VALIDATION
    // ============================================================

    reg valid_instruction_id;

    always @(*) begin

        valid_instruction_id = valid_major_id;

        case (opcode_id)

            `OPCODE_OP: begin

                if (funct7_id == 7'b0000001) begin
                    valid_instruction_id = 1'b1;
                end
                else begin

                    case (funct3_id)

                        3'b000:
                            valid_instruction_id =
                                (funct7_id == 7'b0000000) ||
                                (funct7_id == 7'b0100000);

                        3'b001,
                        3'b010,
                        3'b011,
                        3'b100,
                        3'b110,
                        3'b111:
                            valid_instruction_id =
                                (funct7_id == 7'b0000000);

                        3'b101:
                            valid_instruction_id =
                                (funct7_id == 7'b0000000) ||
                                (funct7_id == 7'b0100000);

                        default:
                            valid_instruction_id = 1'b0;

                    endcase

                end

            end

            `OPCODE_OP_IMM: begin

                case (funct3_id)

                    3'b000,
                    3'b010,
                    3'b011,
                    3'b100,
                    3'b110,
                    3'b111:
                        valid_instruction_id = 1'b1;

                    3'b001:
                        valid_instruction_id =
                            (instruction_id[31:25] == 7'b0000000);

                    3'b101:
                        valid_instruction_id =
                            (instruction_id[31:25] == 7'b0000000) ||
                            (instruction_id[31:25] == 7'b0100000);

                    default:
                        valid_instruction_id = 1'b0;

                endcase

            end

            `OPCODE_LOAD:
                valid_instruction_id =
                    (funct3_id == 3'b000) ||
                    (funct3_id == 3'b001) ||
                    (funct3_id == 3'b010) ||
                    (funct3_id == 3'b100) ||
                    (funct3_id == 3'b101);

            `OPCODE_STORE:
                valid_instruction_id =
                    (funct3_id == 3'b000) ||
                    (funct3_id == 3'b001) ||
                    (funct3_id == 3'b010);

            `OPCODE_BRANCH:
                valid_instruction_id =
                    (funct3_id == 3'b000) ||
                    (funct3_id == 3'b001) ||
                    (funct3_id == 3'b100) ||
                    (funct3_id == 3'b101) ||
                    (funct3_id == 3'b110) ||
                    (funct3_id == 3'b111);

            `OPCODE_JALR:
                valid_instruction_id =
                    (funct3_id == 3'b000);

            `OPCODE_SYSTEM: begin

                if (funct3_id == 3'b000)

                    valid_instruction_id =
                        (instruction_id == 32'h00000073) ||
                        (instruction_id == 32'h00100073) ||
                        (instruction_id == 32'h30200073);

                else

                    valid_instruction_id =
                        (funct3_id == 3'b001) ||
                        (funct3_id == 3'b010) ||
                        (funct3_id == 3'b011) ||
                        (funct3_id == 3'b101) ||
                        (funct3_id == 3'b110) ||
                        (funct3_id == 3'b111);

            end

            default: begin
            end

        endcase

    end

    // ============================================================
    // REGISTER FILE
    // ============================================================

    wire [31:0] register_data1_id;
    wire [31:0] register_data2_id;

    wire reg_write_wb;
    wire [4:0] rd_wb;
    wire [31:0] wb_result;

    reg_file REGISTER_FILE (
        .clk(clk),
        .reset(reset),

        .rs1(rs1_id),
        .rs2(rs2_id),

        .rd(rd_wb),

        .write_data(wb_result),

        .reg_write(reg_write_wb),

        .read_data1(register_data1_id),
        .read_data2(register_data2_id)
    );

    // ============================================================
    // HAZARD UNIT
    // ============================================================

    reg mem_read_ex;
    reg [4:0] rd_ex;

    hazard_unit HAZARD_UNIT (
        .mem_read_ex(mem_read_ex),
        .rd_ex(rd_ex),

        .rs1_id(rs1_id),
        .rs2_id(rs2_id),

        .opcode_id(opcode_id),

        .stall(stall)
    );

    // ============================================================
    // ID / EX PIPELINE REGISTER
    // ============================================================

    reg [31:0] instruction_ex;

    reg [31:0] reg_data1_ex;
    reg [31:0] reg_data2_ex;

    reg [31:0] immediate_ex;

    reg [4:0] rs1_ex;
    reg [4:0] rs2_ex;

    reg [2:0] funct3_ex;
    reg [6:0] funct7_ex;
    reg [6:0] opcode_ex;

    reg reg_write_ex;
    reg mem_write_ex;
    reg mem_to_reg_ex;
    reg alu_src_ex;

    reg branch_ex_reg;
    reg jump_ex;
    reg jalr_ex;

    reg lui_ex;
    reg auipc_ex;

    reg csr_ex;

    reg fence_i_ex_reg;

    reg valid_instruction_ex;

    reg predicted_taken_ex;
    reg [31:0] predicted_target_ex;

    always @(posedge clk) begin

        if (ex_stall && !reset) begin

            // Hold: keep the RV32M instruction in EX until the
            // muldiv unit produces its result. A bubble here
            // would throw the instruction away mid-flight.

        end

        else if (reset || pipeline_flush || stall) begin

            pc_ex <= 32'b0;
            instruction_ex <= 32'h00000013;

            reg_data1_ex <= 32'b0;
            reg_data2_ex <= 32'b0;

            immediate_ex <= 32'b0;

            rs1_ex <= 5'b0;
            rs2_ex <= 5'b0;
            rd_ex <= 5'b0;

            funct3_ex <= 3'b0;
            funct7_ex <= 7'b0;
            opcode_ex <= `OPCODE_OP_IMM;

            reg_write_ex <= 1'b0;
            mem_read_ex <= 1'b0;
            mem_write_ex <= 1'b0;
            mem_to_reg_ex <= 1'b0;
            alu_src_ex <= 1'b0;

            branch_ex_reg <= 1'b0;
            jump_ex <= 1'b0;
            jalr_ex <= 1'b0;

            lui_ex <= 1'b0;
            auipc_ex <= 1'b0;

            csr_ex <= 1'b0;

            fence_i_ex_reg <= 1'b0;

            valid_instruction_ex <= 1'b1;

            predicted_taken_ex <= 1'b0;
            predicted_target_ex <= 32'b0;

            // Bubble: not a real instruction
            pipeline_valid_ex <= 1'b0;

        end

        else begin

            pc_ex <= pc_id;
            instruction_ex <= instruction_id;

            reg_data1_ex <= register_data1_id;
            reg_data2_ex <= register_data2_id;

            immediate_ex <= immediate_id;

            rs1_ex <= rs1_id;
            rs2_ex <= rs2_id;
            rd_ex <= rd_id;

            funct3_ex <= funct3_id;
            funct7_ex <= funct7_id;
            opcode_ex <= opcode_id;

            reg_write_ex <= reg_write_id;
            mem_read_ex <= mem_read_id;
            mem_write_ex <= mem_write_id;
            mem_to_reg_ex <= mem_to_reg_id;
            alu_src_ex <= alu_src_id;

            branch_ex_reg <= branch_id;
            jump_ex <= jump_id;
            jalr_ex <= jalr_id;

            lui_ex <= lui_id;
            auipc_ex <= auipc_id;

            csr_ex <= csr_id;

            fence_i_ex_reg <= fence_i_id;

            valid_instruction_ex <= valid_instruction_id;

            predicted_taken_ex <= predicted_taken_id;
            predicted_target_ex <= predicted_target_id;

            pipeline_valid_ex <= valid_id;

        end

    end

    assign branch_ex = branch_ex_reg;
    assign fence_i_ex = fence_i_ex_reg;

// ============================================================
// FORWARDING
// ============================================================

reg reg_write_mem;
reg mem_read_mem_reg;

reg [4:0] rd_mem;

reg [31:0] execute_result_mem;

reg [31:0] execute_result_delay;
reg        reg_write_delay;
reg [4:0]  rd_delay;

wire [1:0] forward_a;
wire [1:0] forward_b;

bypass_unit BYPASS_UNIT (

    .rs1_ex(rs1_ex),
    .rs2_ex(rs2_ex),

    .rd_mem(rd_mem),
    .rd_wb(rd_wb),

    .reg_write_mem(reg_write_mem),
    .reg_write_wb(reg_write_wb),

    .mem_read_mem(mem_read_mem_reg),

    .forward_a(forward_a),
    .forward_b(forward_b)

);

wire forward_delay_a;
wire forward_delay_b;

assign forward_delay_a =
        reg_write_delay &&
        (rd_delay != 5'd0) &&
        (rd_delay == rs1_ex);

assign forward_delay_b =
        reg_write_delay &&
        (rd_delay != 5'd0) &&
        (rd_delay == rs2_ex);

reg [31:0] forwarded_a;
reg [31:0] forwarded_b;

always @(*) begin

    if (forward_a == 2'b10)      forwarded_a = execute_result_mem;
    else if (forward_a == 2'b01) forwarded_a = wb_result;
    else if (forward_delay_a)    forwarded_a = execute_result_delay;
    else                         forwarded_a = reg_data1_ex;

    if (forward_b == 2'b10)      forwarded_b = execute_result_mem;
    else if (forward_b == 2'b01) forwarded_b = wb_result;
    else if (forward_delay_b)    forwarded_b = execute_result_delay;
    else                         forwarded_b = reg_data2_ex;

end

    // ============================================================
    // ALU CONTROL
    // ============================================================

    reg [3:0] alu_control_ex;

    always @(*) begin

        alu_control_ex = `ALU_ADD;

        case (opcode_ex)

            `OPCODE_OP: begin

                if (funct7_ex != 7'b0000001) begin

                    case (funct3_ex)

                        3'b000:
                            alu_control_ex =
                                funct7_ex[5] ? `ALU_SUB : `ALU_ADD;

                        3'b001: alu_control_ex = `ALU_SLL;
                        3'b010: alu_control_ex = `ALU_SLT;
                        3'b011: alu_control_ex = `ALU_SLTU;
                        3'b100: alu_control_ex = `ALU_XOR;

                        3'b101:
                            alu_control_ex =
                                funct7_ex[5] ? `ALU_SRA : `ALU_SRL;

                        3'b110: alu_control_ex = `ALU_OR;
                        3'b111: alu_control_ex = `ALU_AND;

                    endcase

                end

            end

            `OPCODE_OP_IMM: begin

                case (funct3_ex)

                    3'b000: alu_control_ex = `ALU_ADD;
                    3'b001: alu_control_ex = `ALU_SLL;
                    3'b010: alu_control_ex = `ALU_SLT;
                    3'b011: alu_control_ex = `ALU_SLTU;
                    3'b100: alu_control_ex = `ALU_XOR;

                    3'b101:
                        alu_control_ex =
                            instruction_ex[30] ? `ALU_SRA : `ALU_SRL;

                    3'b110: alu_control_ex = `ALU_OR;
                    3'b111: alu_control_ex = `ALU_AND;

                endcase

            end

            default:
                alu_control_ex = `ALU_ADD;

        endcase

    end

    wire [31:0] alu_operand_b;

    assign alu_operand_b =
        alu_src_ex ? immediate_ex : forwarded_b;

    wire [31:0] alu_result_normal_ex;

    alu_riscv ALU (
        .operand_a(forwarded_a),
        .operand_b(alu_operand_b),
        .alu_control(alu_control_ex),
        .alu_result(alu_result_normal_ex)
    );

    // ============================================================
    // RV32M
    // ============================================================

    wire is_muldiv_ex;

    assign is_muldiv_ex =
        (opcode_ex == `OPCODE_OP) &&
        (funct7_ex == 7'b0000001);

    wire [31:0] muldiv_result_ex;

    muldiv_unit MULDIV_UNIT (
        .clk(clk),
        .reset(reset),
        .req(is_muldiv_ex),
        .operand_a(forwarded_a),
        .operand_b(forwarded_b),
        .funct3(funct3_ex),
        .result(muldiv_result_ex),
        .busy(muldiv_busy)
    );

    // ============================================================
    // BRANCH UNIT
    // ============================================================

    branch_unit BRANCH_UNIT (
        .operand_a(forwarded_a),
        .operand_b(forwarded_b),
        .funct3(funct3_ex),
        .branch_taken(actual_branch_taken_ex)
    );

    wire [31:0] branch_target_ex;

    assign branch_target_ex = pc_ex + immediate_ex;

    wire [31:0] jump_target_ex;

    assign jump_target_ex =
        jalr_ex ?
        ((forwarded_a + immediate_ex) & 32'hFFFFFFFE) :
        (pc_ex + immediate_ex);

    // ============================================================
    // SYSTEM DECODING
    // ============================================================

    wire system_ex;
    assign system_ex = (opcode_ex == `OPCODE_SYSTEM);

    wire csr_instruction_ex;
    assign csr_instruction_ex = system_ex && (funct3_ex != 3'b000);

    wire ecall_ex;
    assign ecall_ex = system_ex && (instruction_ex == 32'h00000073);

    wire ebreak_ex;
    assign ebreak_ex = system_ex && (instruction_ex == 32'h00100073);

    assign mret_ex = system_ex && (instruction_ex == 32'h30200073);

    // ============================================================
    // CSR
    // ============================================================

    wire [31:0] csr_read_data_ex;

    wire [31:0] exception_cause_ex;
    wire [31:0] exception_value_ex;

    // Machine external interrupt: cause = 0x8000_000B
    wire [31:0] trap_cause_final;
    wire [31:0] trap_value_final;

    assign trap_cause_final =
        irq_take ? 32'h8000000B : exception_cause_ex;

    assign trap_value_final =
        irq_take ? 32'b0 : exception_value_ex;

    csr_unit CSR_UNIT (
        .clk(clk),
        .reset(reset),

        .csr_enable(csr_instruction_ex && !trap_ex),

        .csr_funct3(funct3_ex),

        .csr_address(instruction_ex[31:20]),

        .rs1_value(forwarded_a),

        .zimm(instruction_ex[19:15]),

        .csr_read_data(csr_read_data_ex),

        .trap_enter(trap_ex),

        .trap_pc(pc_ex),

        .trap_cause(trap_cause_final),

        .trap_value(trap_value_final),

        .mret(mret_ex),

        .mtvec_out(mtvec),

        .mepc_out(mepc),

        .irq_external(irq_external),

        .interrupt_request(interrupt_request)
    );

    // ============================================================
    // EXECUTION RESULT
    // ============================================================

    wire [31:0] execute_result_ex;

    assign execute_result_ex =
        csr_instruction_ex ? csr_read_data_ex :
        is_muldiv_ex       ? muldiv_result_ex :
        lui_ex             ? immediate_ex :
        auipc_ex           ? (pc_ex + immediate_ex) :
        jump_ex            ? (pc_ex + 32'd4) :
                             alu_result_normal_ex;

    // ============================================================
    // EXCEPTION UNIT
    // ============================================================

    exception_unit EXCEPTION_UNIT (
        .pc(pc_ex),
        .instruction(instruction_ex),
        .valid_instruction(valid_instruction_ex),
        .ecall(ecall_ex),
        .ebreak(ebreak_ex),
        .mem_read(mem_read_ex),
        .mem_write(mem_write_ex),
        .mem_funct3(funct3_ex),
        .memory_address(alu_result_normal_ex),
        .exception(exception_ex),
        .cause(exception_cause_ex),
        .trap_value(exception_value_ex)
    );

    // ============================================================
    // CONTROL REDIRECTION
    // ============================================================

    wire branch_mispredict_ex;

    assign branch_mispredict_ex =
        branch_ex &&
        (
            (predicted_taken_ex != actual_branch_taken_ex)
            ||
            (
                predicted_taken_ex &&
                actual_branch_taken_ex &&
                (predicted_target_ex != branch_target_ex)
            )
        );

    wire jump_redirect_ex;
    assign jump_redirect_ex = jump_ex;

    assign redirect_ex = branch_mispredict_ex | jump_redirect_ex;

    assign recovery_pc_ex =
        jump_ex                ? jump_target_ex :
        actual_branch_taken_ex ? branch_target_ex :
                                 (pc_ex + 32'd4);

    // ============================================================
    // EX / MEM PIPELINE REGISTER
    // ============================================================

    reg mem_write_mem_reg;
    reg mem_to_reg_mem;

    reg [2:0] funct3_mem;

    reg [31:0] store_operand_mem;

    always @(posedge clk) begin

        if (reset) begin

            execute_result_mem <= 32'b0;
            store_operand_mem  <= 32'b0;
            rd_mem             <= 5'b0;
            funct3_mem         <= 3'b0;
            reg_write_mem      <= 1'b0;
            mem_read_mem_reg   <= 1'b0;
            mem_write_mem_reg  <= 1'b0;
            mem_to_reg_mem     <= 1'b0;

        end

        else begin

            execute_result_mem <= execute_result_ex;
            store_operand_mem  <= forwarded_b;
            rd_mem             <= rd_ex;
            funct3_mem         <= funct3_ex;

            // A trapping instruction must not commit, and its
            // memory access must not be issued.
            reg_write_mem <=
                reg_write_ex && !trap_ex && !mret_ex && !ex_stall;

            mem_read_mem_reg <=
                mem_read_ex && !trap_ex && !ex_stall;

            mem_write_mem_reg <=
                mem_write_ex && !trap_ex && !ex_stall;

            mem_to_reg_mem <= mem_to_reg_ex;

        end

    end

// ============================================================
// STORE UNIT
// ============================================================

wire [31:0] formatted_store_data;
wire [3:0]  write_mask;

store_unit STORE_UNIT (
    .funct3(funct3_mem),
    .address_offset(execute_result_mem[1:0]),
    .rs2_data(store_operand_mem),
    .store_data(formatted_store_data),
    .write_mask(write_mask)
);

// ============================================================
// EXTERNAL MEMORY / PERIPHERAL PORT
//
// Same contract the old data_mem had:
//   request presented in cycle N, read data valid in cycle N+1.
// ============================================================

assign dmem_addr  = execute_result_mem;
assign dmem_wdata = formatted_store_data;
assign dmem_wstrb = write_mask;
assign dmem_we    = mem_write_mem_reg;
assign dmem_re    = mem_read_mem_reg;

wire [31:0] memory_read_data;
assign memory_read_data = dmem_rdata;

// ============================================================
// SYNCHRONOUS MEMORY REQUEST PIPELINE
// ============================================================

reg [31:0] load_address_delay;
reg [2:0]  load_funct3_delay;

reg        mem_to_reg_delay;

always @(posedge clk) begin

    if (reset) begin

        load_address_delay   <= 32'b0;
        load_funct3_delay    <= 3'b0;
        execute_result_delay <= 32'b0;
        mem_to_reg_delay     <= 1'b0;
        reg_write_delay      <= 1'b0;
        rd_delay             <= 5'b0;

    end

    else begin

        load_address_delay   <= execute_result_mem;
        load_funct3_delay    <= funct3_mem;
        execute_result_delay <= execute_result_mem;
        mem_to_reg_delay     <= mem_to_reg_mem;
        reg_write_delay      <= reg_write_mem;
        rd_delay             <= rd_mem;

    end

end

// ============================================================
// LOAD UNIT
// ============================================================

wire [31:0] formatted_load_data;

load_unit LOAD_UNIT (
    .memory_data(memory_read_data),
    .address_offset(load_address_delay[1:0]),
    .funct3(load_funct3_delay),
    .load_data(formatted_load_data)
);

// ============================================================
// MEM / WB PIPELINE REGISTER
// ============================================================

reg [31:0] execute_result_wb;
reg [31:0] load_data_wb;

reg mem_to_reg_wb;
reg reg_write_wb_reg;
reg [4:0] rd_wb_reg;

always @(posedge clk) begin

    if (reset) begin

        execute_result_wb <= 32'b0;
        load_data_wb      <= 32'b0;
        mem_to_reg_wb     <= 1'b0;
        reg_write_wb_reg  <= 1'b0;
        rd_wb_reg         <= 5'b0;

    end

    else begin

        execute_result_wb <= execute_result_delay;
        load_data_wb      <= formatted_load_data;
        mem_to_reg_wb     <= mem_to_reg_delay;
        reg_write_wb_reg  <= reg_write_delay;
        rd_wb_reg         <= rd_delay;

    end

end

    assign rd_wb        = rd_wb_reg;
    assign reg_write_wb = reg_write_wb_reg;

    assign wb_result =
        mem_to_reg_wb ? load_data_wb : execute_result_wb;

// ============================================================
// DEBUG / FPGA OBSERVABILITY OUTPUTS
// ============================================================

assign debug_pc            = pc;
assign debug_instruction   = instruction_if;
assign debug_alu_result    = execute_result_ex;
assign debug_wb_data       = wb_result;
assign debug_wb_rd         = rd_wb;
assign debug_reg_write     = reg_write_wb;
assign debug_stall         = stall;
assign debug_branch_taken  = branch_ex && actual_branch_taken_ex;
assign debug_irq_taken     = irq_take;
assign debug_muldiv_busy   = ex_stall;

endmodule
