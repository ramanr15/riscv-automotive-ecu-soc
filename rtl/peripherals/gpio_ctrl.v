`timescale 1ns / 1ps
// ============================================================
// gpio_ctrl : 32-pin GPIO with edge-triggered interrupts
// Person 2 - Step 7
//
// REGISTER MAP
//   0x00  DIR       RW  1 = output, 0 = input
//   0x04  OUT       RW  output value (drives pins in output mode)
//   0x08  IN        RO  synchronised pin value
//   0x0C  RISE_EN   RW  per-pin rising edge detect enable
//   0x10  FALL_EN   RW  per-pin falling edge detect enable
//   0x14  IRQ_PEND  RW  write 1 to clear
//   0x18  IRQ_EN    RW  per-pin contribution to the IRQ output
// ============================================================
module gpio_ctrl #(
    parameter WIDTH = 32
)(
    input  wire              clk,
    input  wire              reset,

    input  wire              sel,
    input  wire [31:0]       addr,
    input  wire [31:0]       wdata,
    input  wire              we,
    input  wire              re,
    output reg  [31:0]       prdata,

    // Pad interface (split, tri-state resolved at the top level)
    input  wire [WIDTH-1:0]  gpio_in,
    output wire [WIDTH-1:0]  gpio_out,
    output wire [WIDTH-1:0]  gpio_oe,

    output wire              irq
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg [WIDTH-1:0] dir;
    reg [WIDTH-1:0] out_val;
    reg [WIDTH-1:0] rise_en;
    reg [WIDTH-1:0] fall_en;
    reg [WIDTH-1:0] irq_pend;
    reg [WIDTH-1:0] irq_en;

    assign gpio_out = out_val;
    assign gpio_oe  = dir;

    // ------------------------------------------------------------
    // Two-flop synchroniser on every input pin.
    // The raw pad is asynchronous, so it must never be used
    // directly in edge detect or any logic path.
    // ------------------------------------------------------------
    reg [WIDTH-1:0] sync0;
    reg [WIDTH-1:0] sync1;
    reg [WIDTH-1:0] sync1_q;

    always @(posedge clk) begin
        if (reset) begin
            sync0   <= {WIDTH{1'b0}};
            sync1   <= {WIDTH{1'b0}};
            sync1_q <= {WIDTH{1'b0}};
        end
        else begin
            sync0   <= gpio_in;
            sync1   <= sync0;
            sync1_q <= sync1;
        end
    end

    wire [WIDTH-1:0] edge_rise;
    wire [WIDTH-1:0] edge_fall;

    assign edge_rise =  sync1 & ~sync1_q;
    assign edge_fall = ~sync1 &  sync1_q;

    wire [WIDTH-1:0] edge_hit;
    assign edge_hit = (edge_rise & rise_en) | (edge_fall & fall_en);

    // ------------------------------------------------------------
    // Register writes
    // ------------------------------------------------------------
    integer b;
    always @(posedge clk) begin
        if (reset) begin
            dir      <= {WIDTH{1'b0}};
            out_val  <= {WIDTH{1'b0}};
            rise_en  <= {WIDTH{1'b0}};
            fall_en  <= {WIDTH{1'b0}};
            irq_en   <= {WIDTH{1'b0}};
            irq_pend <= {WIDTH{1'b0}};
        end
        else begin
            if (wr) begin
                case (reg_index)
                    6'h00: dir     <= wdata[WIDTH-1:0];
                    6'h01: out_val <= wdata[WIDTH-1:0];
                    6'h03: rise_en <= wdata[WIDTH-1:0];
                    6'h04: fall_en <= wdata[WIDTH-1:0];
                    6'h06: irq_en  <= wdata[WIDTH-1:0];
                    default: ;
                endcase
            end

            // Pending: hardware set wins over software clear.
            for (b = 0; b < WIDTH; b = b + 1) begin
                if (edge_hit[b])
                    irq_pend[b] <= 1'b1;
                else if (wr && (reg_index == 6'h05) && wdata[b])
                    irq_pend[b] <= 1'b0;
            end
        end
    end

    assign irq = |(irq_pend & irq_en);

    // ------------------------------------------------------------
    // Read port
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset)
            prdata <= 32'b0;
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= dir;
                6'h01:   prdata <= out_val;
                6'h02:   prdata <= sync1;
                6'h03:   prdata <= rise_en;
                6'h04:   prdata <= fall_en;
                6'h05:   prdata <= irq_pend;
                6'h06:   prdata <= irq_en;
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
