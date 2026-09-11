`include "Def.v"

module exception_unit (

    input wire [31:0] pc,

    input wire [31:0] instruction,

    input wire valid_instruction,

    input wire ecall,

    input wire ebreak,

    input wire mem_read,

    input wire mem_write,

    input wire [2:0] mem_funct3,

    input wire [31:0] memory_address,

    output reg exception,

    output reg [31:0] cause,

    output reg [31:0] trap_value

);


reg load_misaligned;
reg store_misaligned;


always @(*) begin

    load_misaligned = 1'b0;
    store_misaligned = 1'b0;


    // ============================================================
    // LOAD ALIGNMENT
    // ============================================================

    if (mem_read) begin

        case (mem_funct3)

            // LB / LBU
            3'b000,
            3'b100:
                load_misaligned = 1'b0;


            // LH / LHU
            3'b001,
            3'b101:
                load_misaligned =
                    memory_address[0];


            // LW
            3'b010:
                load_misaligned =
                    |memory_address[1:0];


            default:
                load_misaligned = 1'b0;

        endcase

    end


    // ============================================================
    // STORE ALIGNMENT
    // ============================================================

    if (mem_write) begin

        case (mem_funct3)

            // SB
            3'b000:
                store_misaligned = 1'b0;


            // SH
            3'b001:
                store_misaligned =
                    memory_address[0];


            // SW
            3'b010:
                store_misaligned =
                    |memory_address[1:0];


            default:
                store_misaligned = 1'b0;

        endcase

    end

end


// ============================================================
// EXCEPTION PRIORITY
// ============================================================

always @(*) begin

    exception  = 1'b0;

    cause      = 32'b0;

    trap_value = 32'b0;


    // Instruction-address misalignment
    if (|pc[1:0]) begin

        exception  = 1'b1;

        cause      =
            `CAUSE_INST_MISALIGNED;

        trap_value = pc;

    end


    // Illegal instruction
    else if (!valid_instruction) begin

        exception = 1'b1;

        cause =
            `CAUSE_ILLEGAL_INST;

        trap_value =
            instruction;

    end


    // EBREAK
    else if (ebreak) begin

        exception = 1'b1;

        cause =
            `CAUSE_BREAKPOINT;

        trap_value = 32'b0;

    end


    // Load address misalignment
    else if (load_misaligned) begin

        exception = 1'b1;

        cause =
            `CAUSE_LOAD_MISALIGNED;

        trap_value =
            memory_address;

    end


    // Store address misalignment
    else if (store_misaligned) begin

        exception = 1'b1;

        cause =
            `CAUSE_STORE_MISALIGNED;

        trap_value =
            memory_address;

    end


    // ECALL
    else if (ecall) begin

        exception = 1'b1;

        cause =
            `CAUSE_ECALL_MMODE;

        trap_value = 32'b0;

    end

end

endmodule