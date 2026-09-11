`include "Def.v"

module control_unit (

    input wire [6:0] opcode,
    input wire [2:0] funct3,

    output reg reg_write,

    output reg mem_read,
    output reg mem_write,

    output reg mem_to_reg,

    output reg alu_src,

    output reg branch,

    output reg jump,
    output reg jalr,

    output reg lui,
    output reg auipc,

    output reg system,
    output reg csr,

    output reg fence,
    output reg fence_i,

    output reg valid_instruction

);


always @(*) begin

    reg_write = 1'b0;

    mem_read = 1'b0;
    mem_write = 1'b0;

    mem_to_reg = 1'b0;

    alu_src = 1'b0;

    branch = 1'b0;

    jump = 1'b0;
    jalr = 1'b0;

    lui = 1'b0;
    auipc = 1'b0;

    system = 1'b0;
    csr = 1'b0;

    fence = 1'b0;
    fence_i = 1'b0;

    valid_instruction = 1'b1;


    case (opcode)

        // ========================================================
        // R TYPE / RV32M
        // ========================================================

        `OPCODE_OP: begin

            reg_write = 1'b1;

        end


        // ========================================================
        // I TYPE ALU
        // ========================================================

        `OPCODE_OP_IMM: begin

            reg_write = 1'b1;

            alu_src = 1'b1;

        end


        // ========================================================
        // LOAD
        // ========================================================

        `OPCODE_LOAD: begin

            reg_write = 1'b1;

            mem_read = 1'b1;

            mem_to_reg = 1'b1;

            alu_src = 1'b1;

        end


        // ========================================================
        // STORE
        // ========================================================

        `OPCODE_STORE: begin

            mem_write = 1'b1;

            alu_src = 1'b1;

        end


        // ========================================================
        // BRANCH
        // ========================================================

        `OPCODE_BRANCH: begin

            branch = 1'b1;

        end


        // ========================================================
        // JAL
        // ========================================================

        `OPCODE_JAL: begin

            reg_write = 1'b1;

            jump = 1'b1;

        end


        // ========================================================
        // JALR
        // ========================================================

        `OPCODE_JALR: begin

            reg_write = 1'b1;

            jump = 1'b1;

            jalr = 1'b1;

            alu_src = 1'b1;

        end


        // ========================================================
        // LUI
        // ========================================================

        `OPCODE_LUI: begin

            reg_write = 1'b1;

            lui = 1'b1;

        end


        // ========================================================
        // AUIPC
        // ========================================================

        `OPCODE_AUIPC: begin

            reg_write = 1'b1;

            auipc = 1'b1;

        end


        // ========================================================
        // SYSTEM / CSR
        // ========================================================

        `OPCODE_SYSTEM: begin

            system = 1'b1;

            if (funct3 != 3'b000) begin

                csr = 1'b1;

                reg_write = 1'b1;

            end

        end


        // ========================================================
        // FENCE / FENCE.I
        // ========================================================

        `OPCODE_MISC_MEM: begin

            case (funct3)

                3'b000:
                    fence = 1'b1;

                3'b001:
                    fence_i = 1'b1;

                default:
                    valid_instruction = 1'b0;

            endcase

        end


        // ========================================================
        // INVALID OPCODE
        // ========================================================

        default: begin

            valid_instruction = 1'b0;

        end

    endcase

end

endmodule