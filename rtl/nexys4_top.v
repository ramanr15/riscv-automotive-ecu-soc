`timescale 1ns / 1ps
// ============================================================
// nexys4_top
//
// Board wrapper for the RISC-V Automotive ECU SoC on the
// Nexys 4 / Nexys 4 DDR (XC7A100T-1CSG324C).
//
// Replaces the standalone-CPU wrapper of the same name. The CPU
// is no longer instantiated directly: soc_top now contains the
// CPU, the bus, all eight peripherals, the interrupt controller
// and Person 1's CAN controller.
//
// ------------------------------------------------------------
// CLOCK : 80 MHz, NOT the 75 MHz the old wrapper used and NOT
// the board's raw 100 MHz.
//
//   Post-route timing measured the CPU's fmax at ~93 MHz. The
//   limiting path is register file -> forwarding mux -> ALU ->
//   result mux -> pipeline register, which is inherent to a
//   single-cycle execute stage on a -1 speed grade part.
//   80 MHz leaves ~1.5 ns of margin.
//
//   clk_wiz_0 MUST be reconfigured for an 80.000 MHz clk_out1.
//   Every peripheral divider is derived from this in software
//   (see the CAN test program), so the frequency is the single
//   place it is defined.
//
// ------------------------------------------------------------
// GPIO
//   Inputs come from the 16 slide switches, outputs drive the
//   low 16 LEDs. These are different physical pins, so the GPIO
//   direction register selects which LEDs are actually lit
//   rather than switching a shared pad. A pin configured as an
//   input simply leaves its LED dark.
//
// ------------------------------------------------------------
// I2C
//   Open drain. The RTL only ever pulls low; the pin floats
//   otherwise. EXTERNAL PULL-UPS (typically 4.7k to 3V3) ARE
//   REQUIRED on Pmod JC or the bus will never read high and
//   every transfer will stall waiting for clock stretching.
// ============================================================
module nexys4_top (
    // ---- board clock and reset ----
    input  wire        CLK100MHZ,
    input  wire        BTNC,          // centre button, active high

    // ---- GPIO ----
    input  wire [15:0] SW,            // slide switches -> gpio_in
    output wire [15:0] LED,           // LEDs           <- gpio_out

    // ---- status ----
    output wire [3:0]  LED16_19,      // see assignments below

    // ---- UART over the USB bridge ----
    input  wire        UART_TXD_IN,   // host -> FPGA
    output wire        UART_RXD_OUT,  // FPGA -> host

    // ---- Pmod JA : SPI ----
    output wire        JA1_SPI_SCLK,
    output wire        JA2_SPI_MOSI,
    input  wire        JA3_SPI_MISO,
    output wire [3:0]  JA_SPI_CS_N,

    // ---- Pmod JB : PWM ----
    output wire [3:0]  JB_PWM,

    // ---- Pmod JC : I2C (needs external pull-ups) ----
    inout  wire        JC1_I2C_SCL,
    inout  wire        JC2_I2C_SDA,

    // ---- Pmod JD : CAN transceiver ----
    input  wire        JD1_CAN_RX,
    output wire        JD2_CAN_TX
);

    // ------------------------------------------------------------
    // Clock generation : 100 MHz board -> 80 MHz SoC
    // ------------------------------------------------------------
    wire cpu_clk;
    wire clk_locked;

    clk_wiz_0 CLOCK_GENERATOR (
        .clk_in1  (CLK100MHZ),
        .reset    (1'b0),
        .clk_out1 (cpu_clk),
        .locked   (clk_locked)
    );

    // ------------------------------------------------------------
    // Reset
    //
    // Held while the button is pressed or the MMCM has not locked,
    // then synchronised to cpu_clk and stretched. The SoC reset is
    // synchronous, so releasing it asynchronously would let
    // different registers leave reset on different edges.
    // ------------------------------------------------------------
    wire raw_reset;
    assign raw_reset = BTNC | ~clk_locked;

    // Powers up asserted. The MMCM output is not guaranteed to be
    // toggling before `locked` rises, so if these flip-flops came
    // out of configuration at 0 the SoC could start with no reset
    // pulse at all. Vivado turns this initial value into an INIT
    // attribute on the flip-flops, so it costs nothing.
    reg [3:0] reset_sync = 4'b1111;

    always @(posedge cpu_clk) begin
        if (raw_reset)
            reset_sync <= 4'b1111;
        else
            reset_sync <= {reset_sync[2:0], 1'b0};
    end

    wire cpu_reset;
    assign cpu_reset = reset_sync[3];

    // ------------------------------------------------------------
    // Pad-level signals
    // ------------------------------------------------------------
    wire [15:0] gpio_in;
    wire [15:0] gpio_out;
    wire [15:0] gpio_oe;

    wire        i2c_scl_low;
    wire        i2c_sda_low;

    // GPIO inputs: switches, double-flopped inside gpio_ctrl.
    assign gpio_in = SW;

    // GPIO outputs: an LED lights only when its pin is configured
    // as an output AND driven high.
    assign LED = gpio_out & gpio_oe;

    // I2C open drain. External pull-ups required.
    assign JC1_I2C_SCL = i2c_scl_low ? 1'b0 : 1'bz;
    assign JC2_I2C_SDA = i2c_sda_low ? 1'b0 : 1'bz;

    // ------------------------------------------------------------
    // Debug / status
    // ------------------------------------------------------------
    (* mark_debug = "true" *) wire [31:0] debug_pc;
    (* mark_debug = "true" *) wire [31:0] debug_instruction;
    (* mark_debug = "true" *) wire [31:0] debug_wb_data;
    (* mark_debug = "true" *) wire        debug_irq_taken;
    (* mark_debug = "true" *) wire        bus_error;

    // An interrupt is a single cycle at 80 MHz -- far too short to
    // see. Stretch it to roughly 0.2 s so the LED actually blinks.
    reg [23:0] irq_stretch;

    always @(posedge cpu_clk) begin
        if (cpu_reset)
            irq_stretch <= 24'd0;
        else if (debug_irq_taken)
            irq_stretch <= 24'hFF_FFFF;
        else if (irq_stretch != 24'd0)
            irq_stretch <= irq_stretch - 24'd1;
    end

    // A bus error means software touched an unmapped address.
    // Latch it: it must not be possible to miss.
    reg bus_error_latched;

    always @(posedge cpu_clk) begin
        if (cpu_reset)
            bus_error_latched <= 1'b0;
        else if (bus_error)
            bus_error_latched <= 1'b1;
    end

    assign LED16_19[0] = clk_locked;              // MMCM locked
    assign LED16_19[1] = ~cpu_reset;              // SoC running
    assign LED16_19[2] = (irq_stretch != 24'd0);  // interrupt taken
    assign LED16_19[3] = bus_error_latched;       // BAD: unmapped access

    // ------------------------------------------------------------
    // The SoC
    // ------------------------------------------------------------
    soc_top #(
        .GPIO_WIDTH  (16),
        .PWM_CHANNELS(4),
        .SPI_CS      (4)
    ) SOC (
        .clk        (cpu_clk),
        .reset      (cpu_reset),

        .gpio_in    (gpio_in),
        .gpio_out   (gpio_out),
        .gpio_oe    (gpio_oe),

        .uart_rx    (UART_TXD_IN),
        .uart_tx    (UART_RXD_OUT),

        .spi_sclk   (JA1_SPI_SCLK),
        .spi_mosi   (JA2_SPI_MOSI),
        .spi_miso   (JA3_SPI_MISO),
        .spi_cs_n   (JA_SPI_CS_N),

        .i2c_scl_i  (JC1_I2C_SCL),
        .i2c_sda_i  (JC2_I2C_SDA),
        .i2c_scl_low(i2c_scl_low),
        .i2c_sda_low(i2c_sda_low),

        .pwm_out    (JB_PWM),

        // No ADC hardware attached yet. The XADC needs a DRP
        // wrapper; until then the driver's busy/done never fire
        // and software simply must not poll for a conversion.
        .adc_start  (),
        .adc_channel(),
        .adc_busy   (1'b0),
        .adc_done   (1'b0),
        .adc_data   (16'b0),

        .can_rx     (JD1_CAN_RX),
        .can_tx     (JD2_CAN_TX),

        .debug_pc         (debug_pc),
        .debug_instruction(debug_instruction),
        .debug_wb_data    (debug_wb_data),
        .debug_irq_taken  (debug_irq_taken),
        .bus_error        (bus_error)
    );

endmodule
