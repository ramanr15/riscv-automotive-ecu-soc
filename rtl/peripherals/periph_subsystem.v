`timescale 1ns / 1ps
// ============================================================
// periph_subsystem : bus + data memory + all peripherals
// Person 2 - integration
//
// This is the single block that sits between the CPU's MEM
// stage and the outside world. The CAN controller from
// Person 1 is not instantiated here: its bus port and its
// three interrupt lines are brought out so it can be dropped
// in at the SoC top level without touching this file.
//
// SLAVE INDEX = address bits [19:16], peripheral base 0x4000_0000
//   0  GPIO   0x4000_0000
//   1  TIMER  0x4001_0000
//   2  PWM    0x4002_0000
//   3  UART   0x4003_0000
//   4  SPI    0x4004_0000
//   5  I2C    0x4005_0000
//   6  ADC    0x4006_0000
//   7  CAN    0x4007_0000   (Person 1, external port)
//   8  INTC   0x4008_0000
// ============================================================
module periph_subsystem #(
    parameter GPIO_WIDTH   = 32,
    parameter PWM_CHANNELS = 4,
    parameter SPI_CS       = 4
)(
    input  wire                    clk,
    input  wire                    reset,

    // ---------------- CPU memory port ----------------
    input  wire [31:0]             cpu_addr,
    input  wire [31:0]             cpu_wdata,
    input  wire [3:0]              cpu_wstrb,
    input  wire                    cpu_we,
    input  wire                    cpu_re,
    output wire [31:0]             cpu_rdata,
    output wire                    bus_error,

    // ---------------- CPU interrupt ----------------
    output wire                    irq_external,

    // ---------------- GPIO pads ----------------
    input  wire [GPIO_WIDTH-1:0]   gpio_in,
    output wire [GPIO_WIDTH-1:0]   gpio_out,
    output wire [GPIO_WIDTH-1:0]   gpio_oe,

    // ---------------- UART pads ----------------
    input  wire                    uart_rx,
    output wire                    uart_tx,

    // ---------------- SPI pads ----------------
    output wire                    spi_sclk,
    output wire                    spi_mosi,
    input  wire                    spi_miso,
    output wire [SPI_CS-1:0]       spi_cs_n,

    // ---------------- I2C pads (open drain) ----------------
    input  wire                    i2c_scl_i,
    input  wire                    i2c_sda_i,
    output wire                    i2c_scl_low,
    output wire                    i2c_sda_low,

    // ---------------- PWM pads ----------------
    output wire [PWM_CHANNELS-1:0] pwm_out,

    // ---------------- ADC converter interface ----------------
    output wire                    adc_start,
    output wire [3:0]              adc_channel,
    input  wire                    adc_busy,
    input  wire                    adc_done,
    input  wire [15:0]             adc_data,

    // ---------------- CAN slave port (Person 1) ----------------
    output wire                    can_sel,
    output wire [31:0]             can_addr,
    output wire [31:0]             can_wdata,
    output wire [3:0]              can_wstrb,
    output wire                    can_we,
    output wire                    can_re,
    input  wire [31:0]             can_rdata,
    input  wire                    can_irq_rx,
    input  wire                    can_irq_tx,
    input  wire                    can_irq_err
);

    localparam NUM_SLAVES = 9;

    // ------------------------------------------------------------
    // Bus fabric
    // ------------------------------------------------------------
    wire                    dmem_sel;
    wire [31:0]             dmem_rdata;

    wire [NUM_SLAVES-1:0]   s_sel;
    wire [31:0]             s_addr;
    wire [31:0]             s_wdata;
    wire [3:0]              s_wstrb;
    wire                    s_we;
    wire                    s_re;

    wire [31:0] rd_gpio, rd_timer, rd_pwm, rd_uart;
    wire [31:0] rd_spi,  rd_i2c,   rd_adc, rd_intc;

    wire [NUM_SLAVES*32-1:0] s_rdata;

    assign s_rdata = { rd_intc,      // 8 INTC
                       can_rdata,    // 7 CAN
                       rd_adc,       // 6
                       rd_i2c,       // 5
                       rd_spi,       // 4
                       rd_uart,      // 3
                       rd_pwm,       // 2
                       rd_timer,     // 1
                       rd_gpio };    // 0

    bus_interconnect #(.NUM_SLAVES(NUM_SLAVES)) BUS (
        .clk       (clk),
        .reset     (reset),
        .cpu_addr  (cpu_addr),
        .cpu_wdata (cpu_wdata),
        .cpu_wstrb (cpu_wstrb),
        .cpu_we    (cpu_we),
        .cpu_re    (cpu_re),
        .cpu_rdata (cpu_rdata),
        .bus_error (bus_error),
        .dmem_sel  (dmem_sel),
        .dmem_rdata(dmem_rdata),
        .s_sel     (s_sel),
        .s_addr    (s_addr),
        .s_wdata   (s_wdata),
        .s_wstrb   (s_wstrb),
        .s_we      (s_we),
        .s_re      (s_re),
        .s_rdata   (s_rdata)
    );

    // ------------------------------------------------------------
    // Data memory (your existing 4 KB block, now decoded)
    // ------------------------------------------------------------
    data_mem DATA_MEMORY (
        .clk       (clk),
        .mem_read  (dmem_sel & cpu_re),
        .mem_write (dmem_sel & cpu_we),
        .address   (cpu_addr),
        .write_data(cpu_wdata),
        .write_mask(cpu_wstrb),
        .read_data (dmem_rdata)
    );

    // ------------------------------------------------------------
    // Peripherals
    // ------------------------------------------------------------
    wire irq_gpio, irq_timer, irq_uart, irq_spi, irq_i2c, irq_adc;

    gpio_ctrl #(.WIDTH(GPIO_WIDTH)) GPIO (
        .clk(clk), .reset(reset),
        .sel(s_sel[0]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_gpio),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe),
        .irq(irq_gpio)
    );

    timer_ctrl TIMER (
        .clk(clk), .reset(reset),
        .sel(s_sel[1]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_timer),
        .irq(irq_timer)
    );

    pwm_ctrl #(.CHANNELS(PWM_CHANNELS)) PWM (
        .clk(clk), .reset(reset),
        .sel(s_sel[2]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_pwm),
        .pwm_out(pwm_out)
    );

    uart_ctrl #(.FIFO_LOG2(4)) UART (
        .clk(clk), .reset(reset),
        .sel(s_sel[3]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_uart),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .irq(irq_uart)
    );

    spi_ctrl #(.NUM_CS(SPI_CS)) SPI (
        .clk(clk), .reset(reset),
        .sel(s_sel[4]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_spi),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
        .irq(irq_spi)
    );

    i2c_ctrl I2C (
        .clk(clk), .reset(reset),
        .sel(s_sel[5]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_i2c),
        .scl_i(i2c_scl_i), .sda_i(i2c_sda_i),
        .scl_low(i2c_scl_low), .sda_low(i2c_sda_low),
        .irq(irq_i2c)
    );

    adc_ctrl ADC (
        .clk(clk), .reset(reset),
        .sel(s_sel[6]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_adc),
        .adc_start(adc_start), .adc_channel(adc_channel),
        .adc_busy(adc_busy), .adc_done(adc_done),
        .adc_data(adc_data),
        .irq(irq_adc)
    );

    // ------------------------------------------------------------
    // CAN slave port passthrough (Person 1 attaches here)
    // ------------------------------------------------------------
    assign can_sel   = s_sel[7];
    assign can_addr  = s_addr;
    assign can_wdata = s_wdata;
    assign can_wstrb = s_wstrb;
    assign can_we    = s_we;
    assign can_re    = s_re;

    // ------------------------------------------------------------
    // Interrupt controller
    // IRQ IDs: 0 CAN_RX, 1 CAN_TX, 2 CAN_ERR, 3 UART,
    //          4 SPI, 5 I2C, 6 TIMER, 7 ADC, 8 GPIO
    // ------------------------------------------------------------
    wire [8:0] irq_sources;

    assign irq_sources = { irq_gpio,      // 8
                           irq_adc,       // 7
                           irq_timer,     // 6
                           irq_i2c,       // 5
                           irq_spi,       // 4
                           irq_uart,      // 3
                           can_irq_err,   // 2
                           can_irq_tx,    // 1
                           can_irq_rx };  // 0

    intc #(.NUM_IRQ(9)) INTC (
        .clk(clk), .reset(reset),
        .sel(s_sel[8]), .addr(s_addr), .wdata(s_wdata),
        .we(s_we), .re(s_re), .prdata(rd_intc),
        .irq_src(irq_sources),
        .irq_out(irq_external)
    );

endmodule
