module load_unit (

    input wire [31:0] memory_data,

    input wire [1:0] address_offset,

    input wire [2:0] funct3,

    output reg [31:0] load_data

);

reg [7:0] selected_byte;

reg [15:0] selected_half;


always @(*) begin

    // ============================================================
    // BYTE SELECTION
    // ============================================================

    case (address_offset)

        2'b00:
            selected_byte =
                memory_data[7:0];

        2'b01:
            selected_byte =
                memory_data[15:8];

        2'b10:
            selected_byte =
                memory_data[23:16];

        2'b11:
            selected_byte =
                memory_data[31:24];

    endcase


    // ============================================================
    // HALFWORD SELECTION
    // ============================================================

    if (address_offset[1] == 1'b0)

        selected_half =
            memory_data[15:0];

    else

        selected_half =
            memory_data[31:16];


    // ============================================================
    // LOAD TYPE
    // ============================================================

    case (funct3)

        // LB
        3'b000:

            load_data = {
                {24{selected_byte[7]}},
                selected_byte
            };


        // LH
        3'b001:

            load_data = {
                {16{selected_half[15]}},
                selected_half
            };


        // LW
        3'b010:

            load_data =
                memory_data;


        // LBU
        3'b100:

            load_data = {
                24'b0,
                selected_byte
            };


        // LHU
        3'b101:

            load_data = {
                16'b0,
                selected_half
            };


        default:

            load_data = 32'b0;

    endcase

end

endmodule