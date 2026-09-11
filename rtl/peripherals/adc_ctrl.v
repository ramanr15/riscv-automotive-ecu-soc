`timescale 1ns / 1ps
// ============================================================
// adc_ctrl : ADC control / status wrapper
// Person 2 - Step 10
//
// This is a wrapper, not a converter. It exposes a
// start-conversion bit, a busy/done status and a result
// register, and talks to whatever converter the board has:
//
//   - Xilinx XADC hard macro: drive adc_start / adc_channel and
//     return adc_done + adc_data from a small XADC DRP wrapper.
//   - External ADC chip over SPI: leave these ports unconnected
//     and use the SPI controller from Step 5 directly rather
//     than duplicating shift-register logic here.
//
// REGISTER MAP
//   0x00  CTRL    RW  [0]    start conversion (self-clearing)
//                     [1]    interrupt enable
//                     [7:4]  channel select
//   0x04  STATUS  RW  [0] busy, [1] done (write 1 to clear)
//   0x08  RESULT  RO  [15:0] last conversion result
// ============================================================
module adc_ctrl (
    input  wire        clk,
    input  wire        reset,

    input  wire        sel,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output reg  [31:0] prdata,

    // Converter interface
    output reg         adc_start,
    output reg  [3:0]  adc_channel,
    input  wire        adc_busy,
    input  wire        adc_done,
    input  wire [15:0] adc_data,

    output wire        irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg        irq_enable;
    reg        busy;
    reg        done;
    reg [15:0] result;

    always @(posedge clk) begin
        if (reset) begin
            adc_start   <= 1'b0;
            adc_channel <= 4'd0;
            irq_enable  <= 1'b0;
            busy        <= 1'b0;
            done        <= 1'b0;
            result      <= 16'd0;
        end
        else begin
            adc_start <= 1'b0;      // one-cycle start pulse

            if (wr) begin
                case (reg_index)
                    6'h00: begin
                        irq_enable  <= wdata[1];
                        adc_channel <= wdata[7:4];
                        if (wdata[0] && !busy) begin
                            adc_start <= 1'b1;
                            busy      <= 1'b1;
                        end
                    end
                    6'h01: if (wdata[1]) done <= 1'b0;
                    default: ;
                endcase
            end

            if (adc_done) begin
                result <= adc_data;
                busy   <= 1'b0;
                done   <= 1'b1;
            end
            else if (adc_busy) begin
                busy <= 1'b1;
            end
        end
    end

    assign irq = done & irq_enable;

    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {24'b0, adc_channel,
                                    2'b0, irq_enable, 1'b0};
                6'h01:   prdata <= {30'b0, done, busy};
                6'h02:   prdata <= {16'b0, result};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
