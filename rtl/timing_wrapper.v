`timescale 1ns / 1ps
// ============================================================
// timing_wrapper
//
// PURPOSE
//   soc_top exposes 260 top-level pins: 96 bits of debug, the
//   72-bit CAN slave port and 21 bits of ADC interface, none of
//   which are connected yet. The 7a100t-csg324 has 210 usable
//   I/O, so Place Design fails before it ever gets to timing.
//
//   This wrapper reduces the pin count to 34 so implementation
//   can run and produce a REAL post-route timing number instead
//   of the post-synthesis estimate.
//
// WHY AN LFSR AND NOT CONSTANTS
//   Tying can_rdata / adc_data to zero would let synthesis
//   constant-propagate straight through the read multiplexer
//   and the ADC path, deleting the logic being measured. The
//   timing report would then look good for the wrong reason.
//
//   Driving them from a free-running LFSR keeps every path
//   alive and unoptimisable. Likewise the wide debug outputs
//   are XOR-reduced into one pin rather than dropped, so the
//   registers feeding them cannot be trimmed away.
//
// THIS IS NOT THE BOARD WRAPPER
//   It exists only to get an honest implementation result.
//   nexys4_top replaces it once the CAN controller lands and
//   the real pinout is agreed.
// ============================================================
module timing_wrapper (
    input  wire        clk,
    input  wire        reset,

    inout  wire [15:0] gpio_pins,

    input  wire        uart_rx,
    output wire        uart_tx,

    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire [3:0]  spi_cs_n,

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    output wire [3:0]  pwm_out,

    // CAN transceiver
    input  wire        can_rx,
    output wire        can_tx,

    // Everything unconnected collapses into this one pin.
    output reg         debug_xor
);

    // ------------------------------------------------------------
    // Pseudo-random stimulus for the otherwise dangling inputs
    // ------------------------------------------------------------
    reg [31:0] lfsr;

    always @(posedge clk) begin
        if (reset)
            lfsr <= 32'hACE1_2345;
        else
            lfsr <= {lfsr[30:0],
                     lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    end

    // ------------------------------------------------------------
    // SoC
    // ------------------------------------------------------------
    wire [31:0] debug_pc;
    wire [31:0] debug_instruction;
    wire [31:0] debug_wb_data;
    wire        debug_irq_taken;
    wire        bus_error;

    wire        adc_start;
    wire [3:0]  adc_channel;

    // soc_top now exposes split signals; the tri-state pads are
    // resolved here, at the true top level.
    wire [15:0] gpio_in;
    wire [15:0] gpio_out;
    wire [15:0] gpio_oe;
    wire        i2c_scl_low;
    wire        i2c_sda_low;

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : GPIO_PADS
            assign gpio_pins[g] = gpio_oe[g] ? gpio_out[g] : 1'bz;
        end
    endgenerate
    assign gpio_in = gpio_pins;

    assign i2c_scl = i2c_scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = i2c_sda_low ? 1'b0 : 1'bz;

    soc_top #(
        .GPIO_WIDTH  (16),
        .PWM_CHANNELS(4),
        .SPI_CS      (4)
    ) SOC (
        .clk        (clk),
        .reset      (reset),

        .gpio_in    (gpio_in),
        .gpio_out   (gpio_out),
        .gpio_oe    (gpio_oe),

        .uart_rx    (uart_rx),
        .uart_tx    (uart_tx),

        .spi_sclk   (spi_sclk),
        .spi_mosi   (spi_mosi),
        .spi_miso   (spi_miso),
        .spi_cs_n   (spi_cs_n),

        .i2c_scl_i  (i2c_scl),
        .i2c_sda_i  (i2c_sda),
        .i2c_scl_low(i2c_scl_low),
        .i2c_sda_low(i2c_sda_low),

        .pwm_out    (pwm_out),

        // ADC: driven, not tied off
        .adc_start  (adc_start),
        .adc_channel(adc_channel),
        .adc_busy   (lfsr[16]),
        .adc_done   (lfsr[17]),
        .adc_data   (lfsr[15:0]),

        // CAN transceiver pins
        .can_rx     (can_rx),
        .can_tx     (can_tx),

        .debug_pc         (debug_pc),
        .debug_instruction(debug_instruction),
        .debug_wb_data    (debug_wb_data),
        .debug_irq_taken  (debug_irq_taken),
        .bus_error        (bus_error)
    );

    // ------------------------------------------------------------
    // Collapse every observed signal into one registered pin.
    // Registered so the reduction tree is not in an I/O path.
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset)
            debug_xor <= 1'b0;
        else
            debug_xor <= ^debug_pc
                       ^ ^debug_instruction
                       ^ ^debug_wb_data
                       ^ debug_irq_taken
                       ^ bus_error
                       ^ adc_start
                       ^ ^adc_channel;
    end

endmodule
