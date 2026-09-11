module reg_file (

    input wire clk,
    input wire reset,

    input wire [4:0] rs1,
    input wire [4:0] rs2,

    input wire [4:0] rd,

    input wire [31:0] write_data,

    input wire reg_write,

    output wire [31:0] read_data1,

    output wire [31:0] read_data2

);

reg [31:0] registers [0:31];

integer i;


always @(posedge clk) begin

    if (reset) begin

        for (i = 0; i < 32; i = i + 1)

            registers[i] <= 32'b0;

    end

    else begin

        if (reg_write && (rd != 5'b0))

            registers[rd] <= write_data;


        // x0 must always remain zero

        registers[0] <= 32'b0;

    end

end


// ============================================================
// WB -> ID WRITE THROUGH
// ============================================================

assign read_data1 =

    (rs1 == 5'b0) ?

        32'b0 :

    ((reg_write &&
      (rd != 5'b0) &&
      (rd == rs1)) ?

        write_data :

        registers[rs1]);


assign read_data2 =

    (rs2 == 5'b0) ?

        32'b0 :

    ((reg_write &&
      (rd != 5'b0) &&
      (rd == rs2)) ?

        write_data :

        registers[rs2]);


endmodule