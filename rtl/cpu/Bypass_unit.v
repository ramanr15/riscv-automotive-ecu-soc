module bypass_unit (

    input wire [4:0] rs1_ex,

    input wire [4:0] rs2_ex,

    input wire [4:0] rd_mem,

    input wire [4:0] rd_wb,

    input wire reg_write_mem,

    input wire reg_write_wb,

    input wire mem_read_mem,

    output reg [1:0] forward_a,

    output reg [1:0] forward_b

);


always @(*) begin

    forward_a = 2'b00;

    forward_b = 2'b00;


    // ============================================================
    // OPERAND A
    // ============================================================

    // EX/MEM forwarding

    if (reg_write_mem &&
        !mem_read_mem &&
        (rd_mem != 5'b0) &&
        (rd_mem == rs1_ex))

        forward_a = 2'b10;


    // MEM/WB forwarding

    else if (reg_write_wb &&
             (rd_wb != 5'b0) &&
             (rd_wb == rs1_ex))

        forward_a = 2'b01;


    // ============================================================
    // OPERAND B
    // ============================================================

    if (reg_write_mem &&
        !mem_read_mem &&
        (rd_mem != 5'b0) &&
        (rd_mem == rs2_ex))

        forward_b = 2'b10;


    else if (reg_write_wb &&
             (rd_wb != 5'b0) &&
             (rd_wb == rs2_ex))

        forward_b = 2'b01;

end

endmodule