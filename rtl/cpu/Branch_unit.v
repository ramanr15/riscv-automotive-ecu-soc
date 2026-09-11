module branch_unit (

    input wire [31:0] operand_a,

    input wire [31:0] operand_b,

    input wire [2:0] funct3,

    output reg branch_taken

);


always @(*) begin

    case (funct3)

        // BEQ
        3'b000:

            branch_taken =
                (operand_a == operand_b);


        // BNE
        3'b001:

            branch_taken =
                (operand_a != operand_b);


        // BLT
        3'b100:

            branch_taken =
                ($signed(operand_a) <
                 $signed(operand_b));


        // BGE
        3'b101:

            branch_taken =
                ($signed(operand_a) >=
                 $signed(operand_b));


        // BLTU
        3'b110:

            branch_taken =
                (operand_a < operand_b);


        // BGEU
        3'b111:

            branch_taken =
                (operand_a >= operand_b);


        default:

            branch_taken = 1'b0;

    endcase

end

endmodule