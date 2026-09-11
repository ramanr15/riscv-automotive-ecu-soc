`timescale 1ns / 1ps
// ============================================================
// intc : Interrupt controller
// Person 2 - Step 2
//
// IRQ ID assignment (Section 1.4)
//   0  CAN RX          (Person 1)
//   1  CAN TX          (Person 1)
//   2  CAN ERROR       (Person 1)
//   3  UART
//   4  SPI
//   5  I2C
//   6  TIMER
//   7  ADC
//   8  GPIO
//
// REGISTER MAP (base + offset)
//   0x00  MASK     RW   one enable bit per source
//   0x04  PENDING  RW   write 1 to clear, latched
//   0x08  CAUSE    RO   lowest pending & enabled IRQ ID
//   0x0C  STATUS   RO   bit0 = any (pending & mask)
//   0x10  RAW      RO   live, unlatched source lines
// ============================================================
module intc #(
    parameter NUM_IRQ = 9
)(
    input  wire                clk,
    input  wire                reset,

    // Bus
    input  wire                sel,
    input  wire [31:0]         addr,
    input  wire [31:0]         wdata,
    input  wire                we,
    input  wire                re,
    output reg  [31:0]         prdata,

    // One request line per source
    input  wire [NUM_IRQ-1:0]  irq_src,

    // Single line into the CPU's external interrupt input
    output wire                irq_out
);

    wire wr;  assign wr = sel & we;
    wire rd;  assign rd = sel & re;

    wire [5:0] reg_index;
    assign reg_index = addr[7:2];

    reg [NUM_IRQ-1:0] mask;
    reg [NUM_IRQ-1:0] pending;

    // ------------------------------------------------------------
    // Edge detect on each source so a short pulse is never lost,
    // and a level-held source does not re-latch every cycle.
    // ------------------------------------------------------------
    reg [NUM_IRQ-1:0] irq_src_q;

    wire [NUM_IRQ-1:0] irq_rise;
    assign irq_rise = irq_src & ~irq_src_q;

    always @(posedge clk) begin
        if (reset) irq_src_q <= {NUM_IRQ{1'b0}};
        else       irq_src_q <= irq_src;
    end

    // ------------------------------------------------------------
    // MASK register
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset)
            mask <= {NUM_IRQ{1'b0}};
        else if (wr && (reg_index == 6'h00))
            mask <= wdata[NUM_IRQ-1:0];
    end

    // ------------------------------------------------------------
    // PENDING register : set by a rising source, cleared by
    // software writing a 1 to that bit. Set wins over clear.
    // ------------------------------------------------------------
    integer p;
    always @(posedge clk) begin
        if (reset) begin
            pending <= {NUM_IRQ{1'b0}};
        end
        else begin
            for (p = 0; p < NUM_IRQ; p = p + 1) begin
                if (irq_rise[p])
                    pending[p] <= 1'b1;
                else if (wr && (reg_index == 6'h01) && wdata[p])
                    pending[p] <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------
    // Fixed priority: lowest IRQ ID wins
    // ------------------------------------------------------------
    wire [NUM_IRQ-1:0] active;
    assign active = pending & mask;

    reg [4:0] cause;
    integer c;
    always @(*) begin
        cause = 5'd31;                       // 31 = no active source
        for (c = NUM_IRQ-1; c >= 0; c = c - 1) begin
            if (active[c]) cause = c[4:0];
        end
    end

    assign irq_out = |active;

    // ------------------------------------------------------------
    // Read port (registered, 1-cycle latency)
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            prdata <= 32'b0;
        end
        else if (rd) begin
            case (reg_index)
                6'h00:   prdata <= {{(32-NUM_IRQ){1'b0}}, mask};
                6'h01:   prdata <= {{(32-NUM_IRQ){1'b0}}, pending};
                6'h02:   prdata <= {27'b0, cause};
                6'h03:   prdata <= {31'b0, irq_out};
                6'h04:   prdata <= {{(32-NUM_IRQ){1'b0}}, irq_src};
                default: prdata <= 32'b0;
            endcase
        end
    end

endmodule
