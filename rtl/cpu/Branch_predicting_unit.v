module branch_predictor (

    input wire clk,

    input wire reset,

    input wire [31:0] fetch_pc,

    input wire update_enable,

    input wire [31:0] update_pc,

    input wire actual_taken,

    output wire predicted_taken

);

reg [1:0] bht [0:15];

integer i;


wire [3:0] fetch_index;

wire [3:0] update_index;


assign fetch_index =
    fetch_pc[5:2];


assign update_index =
    update_pc[5:2];


// MSB determines prediction

assign predicted_taken =
    bht[fetch_index][1];


always @(posedge clk) begin

    if (reset) begin

        for (i = 0; i < 16; i = i + 1)

            // Weakly Not Taken

            bht[i] <= 2'b01;

    end

    else if (update_enable) begin

        case (bht[update_index])

            // Strongly Not Taken
            2'b00: begin

                if (actual_taken)

                    bht[update_index] <= 2'b01;

            end


            // Weakly Not Taken
            2'b01: begin

                if (actual_taken)

                    bht[update_index] <= 2'b10;

                else

                    bht[update_index] <= 2'b00;

            end


            // Weakly Taken
            2'b10: begin

                if (actual_taken)

                    bht[update_index] <= 2'b11;

                else

                    bht[update_index] <= 2'b01;

            end


            // Strongly Taken
            2'b11: begin

                if (!actual_taken)

                    bht[update_index] <= 2'b10;

            end


            default:

                bht[update_index] <= 2'b01;

        endcase

    end

end

endmodule