`timescale 1ns / 1ps
// ============================================================
// soc_top : CPU + peripheral subsystem
//
// This is what nexys4_top should instantiate instead of
// top_level_riscv directly. Tri-state resolution for the I2C
// lines and the bidirectional GPIO pins happens here, which
// keeps every internal module purely synchronous with split
// _i / _o / _oe signals.
//
// Person 1's CAN controller is instantiated here through
// can_adapter, which handles the reset-polarity and read-timing
// mismatches at the boundary.
// ============================================================
module soc_top #(
    parameter GPIO_WIDTH   = 16,
    parameter PWM_CHANNELS = 4,
    parameter SPI_CS       = 4
)(
    input  wire                    clk,
    input  wire                    reset,

    // GPIO, split direction signals.
    //
    // Tri-state resolution happens in the board wrapper, not here.
    // That is where the real pins are known: on Nexys 4 the inputs
    // come from slide switches and the outputs drive LEDs, which
    // are separate pins and must NOT be tied into one inout net.
    // A wrapper targeting true bidirectional pads can still do
    //     assign pad[i] = gpio_oe[i] ? gpio_out[i] : 1'bz;
    input  wire [GPIO_WIDTH-1:0]   gpio_in,
    output wire [GPIO_WIDTH-1:0]   gpio_out,
    output wire [GPIO_WIDTH-1:0]   gpio_oe,

    // UART
    input  wire                    uart_rx,
    output wire                    uart_tx,

    // SPI
    output wire                    spi_sclk,
    output wire                    spi_mosi,
    input  wire                    spi_miso,
    output wire [SPI_CS-1:0]       spi_cs_n,

    // I2C, open drain. Resolved in the board wrapper as
    //     assign scl = scl_low ? 1'b0 : 1'bz;
    input  wire                    i2c_scl_i,
    input  wire                    i2c_sda_i,
    output wire                    i2c_scl_low,
    output wire                    i2c_sda_low,

    // PWM
    output wire [PWM_CHANNELS-1:0] pwm_out,

    // ADC converter hookup (tie off if unused)
    output wire                    adc_start,
    output wire [3:0]              adc_channel,
    input  wire                    adc_busy,
    input  wire                    adc_done,
    input  wire [15:0]             adc_data,

    // CAN transceiver pins (Person 1's controller is now
    // instantiated internally, via can_adapter)
    input  wire                    can_rx,
    output wire                    can_tx,

    // Debug
    output wire [31:0]             debug_pc,
    output wire [31:0]             debug_instruction,
    output wire [31:0]             debug_wb_data,
    output wire                    debug_irq_taken,
    output wire                    bus_error
);

    // ------------------------------------------------------------
    // CPU <-> bus
    // ------------------------------------------------------------
    wire [31:0] dmem_addr;
    wire [31:0] dmem_wdata;
    wire [3:0]  dmem_wstrb;
    wire        dmem_we;
    wire        dmem_re;
    wire [31:0] dmem_rdata;

    wire        irq_external;

    top_level_riscv CPU (
        .clk               (clk),
        .reset             (reset),
        .irq_external      (irq_external),
        .dmem_addr         (dmem_addr),
        .dmem_wdata        (dmem_wdata),
        .dmem_wstrb        (dmem_wstrb),
        .dmem_we           (dmem_we),
        .dmem_re           (dmem_re),
        .dmem_rdata        (dmem_rdata),
        .debug_pc          (debug_pc),
        .debug_instruction (debug_instruction),
        .debug_alu_result  (),
        .debug_wb_data     (debug_wb_data),
        .debug_wb_rd       (),
        .debug_reg_write   (),
        .debug_stall       (),
        .debug_branch_taken(),
        .debug_irq_taken   (debug_irq_taken)
    );

    // ------------------------------------------------------------
    // CAN controller (Person 1)
    //
    // can_adapter reconciles two boundary mismatches: active-low
    // rst_n, and can_regfile.v's combinational read data. See that
    // file's header for the detail.
    // ------------------------------------------------------------
    wire        can_sel;
    wire [31:0] can_addr;
    wire [31:0] can_wdata;
    wire [3:0]  can_wstrb;
    wire        can_we;
    wire        can_re;
    wire [31:0] can_rdata;
    wire        can_irq_rx;
    wire        can_irq_tx;
    wire        can_irq_err;

    can_adapter #(
        .REGISTER_RDATA(1)
    ) CAN (
        .clk    (clk),
        .reset  (reset),

        .sel    (can_sel),
        .addr   (can_addr),
        .wdata  (can_wdata),
        .we     (can_we),
        .re     (can_re),
        .prdata (can_rdata),

        .irq_rx (can_irq_rx),
        .irq_tx (can_irq_tx),
        .irq_err(can_irq_err),

        .can_rx (can_rx),
        .can_tx (can_tx)
    );

    // ------------------------------------------------------------
    // Peripheral subsystem
    // ------------------------------------------------------------
    periph_subsystem #(
        .GPIO_WIDTH  (GPIO_WIDTH),
        .PWM_CHANNELS(PWM_CHANNELS),
        .SPI_CS      (SPI_CS)
    ) PERIPHERALS (
        .clk         (clk),
        .reset       (reset),

        .cpu_addr    (dmem_addr),
        .cpu_wdata   (dmem_wdata),
        .cpu_wstrb   (dmem_wstrb),
        .cpu_we      (dmem_we),
        .cpu_re      (dmem_re),
        .cpu_rdata   (dmem_rdata),
        .bus_error   (bus_error),

        .irq_external(irq_external),

        .gpio_in     (gpio_in),
        .gpio_out    (gpio_out),
        .gpio_oe     (gpio_oe),

        .uart_rx     (uart_rx),
        .uart_tx     (uart_tx),

        .spi_sclk    (spi_sclk),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso),
        .spi_cs_n    (spi_cs_n),

        .i2c_scl_i   (i2c_scl_i),
        .i2c_sda_i   (i2c_sda_i),
        .i2c_scl_low (i2c_scl_low),
        .i2c_sda_low (i2c_sda_low),

        .pwm_out     (pwm_out),

        .adc_start   (adc_start),
        .adc_channel (adc_channel),
        .adc_busy    (adc_busy),
        .adc_done    (adc_done),
        .adc_data    (adc_data),

        .can_sel     (can_sel),
        .can_addr    (can_addr),
        .can_wdata   (can_wdata),
        .can_wstrb   (can_wstrb),
        .can_we      (can_we),
        .can_re      (can_re),
        .can_rdata   (can_rdata),
        .can_irq_rx  (can_irq_rx),
        .can_irq_tx  (can_irq_tx),
        .can_irq_err (can_irq_err)
    );

endmodule
