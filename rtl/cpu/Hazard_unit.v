`include "Def.v"

module hazard_unit (

    input wire mem_read_ex,

    input wire [4:0] rd_ex,

    input wire [4:0] rs1_id,

    input wire [4:0] rs2_id,

    input wire [6:0] opcode_id,

    output reg stall

);

reg uses_rs1;

reg uses_rs2;


always @(*) begin

    uses_rs1 = 1'b0;

    uses_rs2 = 1'b0;


    case (opcode_id)

        // R-type / M-type
        `OPCODE_OP: begin

            uses_rs1 = 1'b1;

            uses_rs2 = 1'b1;

        end


        // I-type ALU
        `OPCODE_OP_IMM: begin

            uses_rs1 = 1'b1;

        end


        // LOAD
        `OPCODE_LOAD: begin

            uses_rs1 = 1'b1;

        end


        // STORE
        `OPCODE_STORE: begin

            uses_rs1 = 1'b1;

            uses_rs2 = 1'b1;

        end


        // BRANCH
        `OPCODE_BRANCH: begin

            uses_rs1 = 1'b1;

            uses_rs2 = 1'b1;

        end


        // JALR
        `OPCODE_JALR: begin

            uses_rs1 = 1'b1;

        end


        // SYSTEM / CSR
        `OPCODE_SYSTEM: begin

            uses_rs1 = 1'b1;

        end


        default: begin

            uses_rs1 = 1'b0;

            uses_rs2 = 1'b0;

        end

    endcase


    stall =

        mem_read_ex &&

        (rd_ex != 5'b0) &&

        (
            (uses_rs1 && (rd_ex == rs1_id))

            ||

            (uses_rs2 && (rd_ex == rs2_id))
        );

end

endmodule