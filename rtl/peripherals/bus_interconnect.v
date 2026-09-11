`timescale 1ns / 1ps
// ============================================================
// bus_interconnect : System bus / address decoder
// Person 2 - Step 1
//
// TIMING CONTRACT (matches the existing data_mem behaviour):
//   cycle N   : master drives addr / we / re / wdata
//   cycle N+1 : slave presents registered read data
//
// The CPU's MEM stage already delays load metadata by one
// cycle, so no ready/stall signal is needed. Every peripheral
// register access is a single cycle. Slow protocol engines
// (SPI / I2C / ADC) run in the background and are polled or
// signalled through interrupts.
//
// MEMORY MAP
//   0x0000_0000 - 0x0FFF_FFFF   Data memory   (aliased 4 KB)
//   0x4000_0000 - 0x4FFF_FFFF   Peripherals, addr[19:16] = slave
//   anything else               unmapped -> bus_error
// ============================================================
module bus_interconnect #(
    parameter NUM_SLAVES = 9
)(
    input  wire                     clk,
    input  wire                     reset,

    // ---------------- CPU (master) side ----------------
    input  wire [31:0]              cpu_addr,
    input  wire [31:0]              cpu_wdata,
    input  wire [3:0]               cpu_wstrb,
    input  wire                     cpu_we,
    input  wire                     cpu_re,
    output reg  [31:0]              cpu_rdata,

    // Asserted for one cycle when an access hit no slave.
    output reg                      bus_error,

    // ---------------- Data memory port ----------------
    output wire                     dmem_sel,
    input  wire [31:0]              dmem_rdata,

    // ---------------- Peripheral fan-out ----------------
    output wire [NUM_SLAVES-1:0]    s_sel,
    output wire [31:0]              s_addr,
    output wire [31:0]              s_wdata,
    output wire [3:0]               s_wstrb,
    output wire                     s_we,
    output wire                     s_re,
    input  wire [NUM_SLAVES*32-1:0] s_rdata
);

    // ------------------------------------------------------------
    // Region decode
    // ------------------------------------------------------------
    wire active;
    assign active = cpu_we | cpu_re;

    wire dmem_region;
    wire periph_region;

    assign dmem_region   = (cpu_addr[31:28] == 4'h0);
    assign periph_region = (cpu_addr[31:28] == 4'h4);

    wire [3:0] slave_index;
    assign slave_index = cpu_addr[19:16];

    assign dmem_sel = active & dmem_region;

    // ------------------------------------------------------------
    // One select line per peripheral (one-hot by construction)
    // ------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_SLAVES; i = i + 1) begin : SEL_DECODE
            assign s_sel[i] = active &
                              periph_region &
                              (slave_index == i);
        end
    endgenerate

    // Address / data / control are broadcast to every peripheral.
    // Only the selected one acts on them.
    assign s_addr  = cpu_addr;
    assign s_wdata = cpu_wdata;
    assign s_wstrb = cpu_wstrb;
    assign s_we    = cpu_we;
    assign s_re    = cpu_re;

    // ------------------------------------------------------------
    // Pipeline the selects by one cycle so they line up with the
    // registered read data coming back from the slaves.
    // ------------------------------------------------------------
    reg [NUM_SLAVES-1:0] s_sel_q;
    reg                  dmem_sel_q;
    reg                  unmapped_q;

    wire hit_any;
    assign hit_any = dmem_sel | (|s_sel);

    always @(posedge clk) begin
        if (reset) begin
            s_sel_q    <= {NUM_SLAVES{1'b0}};
            dmem_sel_q <= 1'b0;
            unmapped_q <= 1'b0;
        end
        else begin
            s_sel_q    <= s_sel;
            dmem_sel_q <= dmem_sel;
            unmapped_q <= active & ~hit_any;
        end
    end

    always @(*) begin
        bus_error = unmapped_q;
    end

    // ------------------------------------------------------------
    // One-hot read-data multiplexer
    // ------------------------------------------------------------
    integer s;
    always @(*) begin
        cpu_rdata = 32'b0;

        if (dmem_sel_q) begin
            cpu_rdata = dmem_rdata;
        end
        else begin
            for (s = 0; s < NUM_SLAVES; s = s + 1) begin
                if (s_sel_q[s])
                    cpu_rdata = s_rdata[s*32 +: 32];
            end
        end
    end

endmodule
