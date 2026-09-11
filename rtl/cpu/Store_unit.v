module store_unit (

    input wire [2:0] funct3,

    input wire [1:0] address_offset,

    input wire [31:0] rs2_data,

    output reg [31:0] store_data,

    output reg [3:0] write_mask

);


always @(*) begin

    store_data = 32'b0;

    write_mask = 4'b0000;


    case (funct3)

        // ========================================================
        // SB
        // ========================================================

        3'b000: begin

            case (address_offset)

                2'b00: begin

                    write_mask =
                        4'b0001;

                    store_data[7:0] =
                        rs2_data[7:0];

                end


                2'b01: begin

                    write_mask =
                        4'b0010;

                    store_data[15:8] =
                        rs2_data[7:0];

                end


                2'b10: begin

                    write_mask =
                        4'b0100;

                    store_data[23:16] =
                        rs2_data[7:0];

                end


                2'b11: begin

                    write_mask =
                        4'b1000;

                    store_data[31:24] =
                        rs2_data[7:0];

                end

            endcase

        end


        // ========================================================
        // SH
        // ========================================================

        3'b001: begin

            if (address_offset[1] == 1'b0) begin

                write_mask =
                    4'b0011;

                store_data[15:0] =
                    rs2_data[15:0];

            end

            else begin

                write_mask =
                    4'b1100;

                store_data[31:16] =
                    rs2_data[15:0];

            end

        end


        // ========================================================
        // SW
        // ========================================================

        3'b010: begin

            write_mask =
                4'b1111;

            store_data =
                rs2_data;

        end


        default: begin

            write_mask =
                4'b0000;

            store_data =
                32'b0;

        end

    endcase

end

endmodule