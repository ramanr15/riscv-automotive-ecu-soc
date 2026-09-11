`timescale 1ns / 1ps
// ============================================================
// timer_ctrl : Bus wrapper around timer_core
// Person 2 - Step 8
//
// REGISTER MAP
//   0x00  CTRL      RW  [0] enable
//                       [1] auto-reload on compare match
//                       [2] interrupt enable
//                       [3] clear counter (self-clearing)
//   0x04  PRESCALE  RW  [15:0] clock divider - 1
//   0x08  COMPARE   RW  compare / period value
//   0x0C  COUNT     RO  live counter value
//   0x10  STATUS    RW  [0] match flag, write 1 to clear
// ============================================================
module timer_ctrl (
    input  wire        clk,
    input  wire        reset,

    input  wire        sel,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output reg  [31:0] prdata,

    output wire        irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg        enable;
    reg        auto_reload;
    reg        irq_enable;
    reg        clear_req;

    reg [15:0] prescale;
    reg [31:0] compare;
    reg        match_flag;

    wire [31:0] count;
    wire        tick;

    timer_core CORE (
        .clk        (clk),
        .reset      (reset),
        .enable     (enable),
        .clear      (clear_req),
        .auto_reload(auto_reload),
        .prescale   (prescale),
        .period     (compare),
        .count      (count),
        .tick       (tick),
        .pre_tick   ()
    );

    always @(posedge clk) begin
        if (reset) begin
            enable      <= 1'b0;
            auto_reload <= 1'b0;
            irq_enable  <= 1'b0;
            clear_req   <= 1'b0;
            prescale    <= 16'd0;
            compare     <= 32'hFFFF_FFFF;
            match_flag  <= 1'b0;
        end
        else begin
            clear_req <= 1'b0;

            if (wr) begin
                case (reg_index)
                    6'h00: begin
                        enable      <= wdata[0];
                        auto_reload <= wdata[1];
                        irq_enable  <= wdata[2];
                        clear_req   <= wdata[3];
                    end
                    6'h01: prescale <= wdata[15:0];
                    6'h02: compare  <= wdata;
                    default: ;
                endcase
            end

            // Status flag: hardware set wins over software clear.
            if (tick)
                match_flag <= 1'b1;
            else if (wr && (reg_index == 6'h04) && wdata[0])
                match_flag <= 1'b0;
        end
    end

    assign irq = match_flag & irq_enable;

    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {28'b0, 1'b0, irq_enable,
                                    auto_reload, enable};
                6'h01:   prdata <= {16'b0, prescale};
                6'h02:   prdata <= compare;
                6'h03:   prdata <= count;
                6'h04:   prdata <= {31'b0, match_flag};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
