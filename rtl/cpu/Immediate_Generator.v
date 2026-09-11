`include "Def.v"

module imm_gen (

    input wire [31:0] instruction,

    output reg [31:0] immediate

);

wire [6:0] opcode;

assign opcode = instruction[6:0];


always @(*) begin

    case (opcode)

        // --------------------------------------------------------
        // I TYPE
        // --------------------------------------------------------

        `OPCODE_OP_IMM,
        `OPCODE_LOAD,
        `OPCODE_JALR:

            immediate = {
                {20{instruction[31]}},
                instruction[31:20]
            };


        // --------------------------------------------------------
        // STORE
        // --------------------------------------------------------

        `OPCODE_STORE:

            immediate = {
                {20{instruction[31]}},
                instruction[31:25],
                instruction[11:7]
            };


        // --------------------------------------------------------
        // BRANCH
        // --------------------------------------------------------

        `OPCODE_BRANCH:

            immediate = {
                {19{instruction[31]}},
                instruction[31],
                instruction[7],
                instruction[30:25],
                instruction[11:8],
                1'b0
            };


        // --------------------------------------------------------
        // U TYPE
        // --------------------------------------------------------

        `OPCODE_LUI,
        `OPCODE_AUIPC:

            immediate = {
                instruction[31:12],
                12'b0
            };


        // --------------------------------------------------------
        // JAL
        // --------------------------------------------------------

        `OPCODE_JAL:

            immediate = {
                {11{instruction[31]}},
                instruction[31],
                instruction[19:12],
                instruction[20],
                instruction[30:21],
                1'b0
            };


        default:

            immediate = 32'b0;

    endcase

end

endmodule