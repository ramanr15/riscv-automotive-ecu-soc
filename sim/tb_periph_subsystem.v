`timescale 1ns / 1ps
// ============================================================
// tb_periph_subsystem
//
// Drives the bus master port directly (no CPU) so the address
// decoder, interrupt controller and every peripheral can be
// verified in isolation, exactly the order Section 3.13 asks
// for. Run this in Vivado's simulator before you ever put the
// CPU behind it.
//
// Covered:
//   1. Address decode + unmapped access -> bus_error
//   2. Data memory read/write through the decoder
//   3. GPIO output, input, edge interrupt
//   4. Timer compare interrupt
//   5. PWM duty output
//   6. UART loopback (tx tied back to rx)
//   7. SPI loopback (mosi tied back to miso)
//   8. Interrupt controller mask / pending / cause
//   9. CAN register access through the real bus  <-- CRITICAL
//  10. CAN loopback frame + interrupt into the INTC
// ============================================================
module tb_periph_subsystem;

    localparam GPIO_WIDTH = 32;

    reg         clk = 1'b0;
    reg         reset = 1'b1;

    always #5 clk = ~clk;              // 100 MHz

    // Bus master
    reg  [31:0] cpu_addr  = 32'b0;
    reg  [31:0] cpu_wdata = 32'b0;
    reg  [3:0]  cpu_wstrb = 4'b0;
    reg         cpu_we    = 1'b0;
    reg         cpu_re    = 1'b0;
    wire [31:0] cpu_rdata;
    wire        bus_error;
    wire        irq_external;

    // Pads
    reg  [GPIO_WIDTH-1:0] gpio_drive = 32'b0;
    wire [GPIO_WIDTH-1:0] gpio_out;
    wire [GPIO_WIDTH-1:0] gpio_oe;
    wire [GPIO_WIDTH-1:0] gpio_in;

    // Pins the DUT does not drive are supplied by the testbench.
    assign gpio_in = (gpio_out & gpio_oe) | (gpio_drive & ~gpio_oe);

    wire uart_tx;
    wire uart_rx;
    assign uart_rx = uart_tx;          // loopback

    wire spi_sclk, spi_mosi;
    wire [3:0] spi_cs_n;
    wire spi_miso;
    assign spi_miso = spi_mosi;        // loopback

    wire i2c_scl_low, i2c_sda_low;
    wire i2c_scl = i2c_scl_low ? 1'b0 : 1'b1;   // pull-ups, no slave
    wire i2c_sda = i2c_sda_low ? 1'b0 : 1'b1;

    wire [3:0] pwm_out;

    wire       adc_start;
    wire [3:0] adc_channel;

    // CAN transceiver, looped back externally as well as
    // internally, so the controller sees its own dominant bits.
    wire can_tx;
    wire can_rx;
    assign can_rx = can_tx;      // external loopback

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

    // ------------------------------------------------------------
    // Base addresses
    // ------------------------------------------------------------
    localparam DMEM  = 32'h0000_0000;
    localparam GPIO  = 32'h4000_0000;
    localparam TIMER = 32'h4001_0000;
    localparam PWM   = 32'h4002_0000;
    localparam UART  = 32'h4003_0000;
    localparam SPI   = 32'h4004_0000;
    localparam I2C   = 32'h4005_0000;
    localparam ADC   = 32'h4006_0000;
    localparam CAN   = 32'h4007_0000;
    localparam INTC  = 32'h4008_0000;

    // CAN register offsets, from Person 1's can_register_map.md
    localparam CAN_CTRL     = 32'h00;
    localparam CAN_BTIME    = 32'h04;
    localparam CAN_STATUS   = 32'h08;
    localparam CAN_TX_ID    = 32'h0C;
    localparam CAN_TX_DATA0 = 32'h10;
    localparam CAN_TX_DATA1 = 32'h14;
    localparam CAN_TX_CTRL  = 32'h18;
    localparam CAN_RX_ID    = 32'h1C;
    localparam CAN_RX_DATA0 = 32'h20;
    localparam CAN_RX_DATA1 = 32'h24;
    localparam CAN_RX_CTRL  = 32'h28;
    localparam CAN_IE       = 32'h2C;
    localparam CAN_IP       = 32'h30;

    integer errors = 0;
    integer can_timeout;

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    periph_subsystem #(
        .GPIO_WIDTH(GPIO_WIDTH),
        .PWM_CHANNELS(4),
        .SPI_CS(4)
    ) DUT (
        .clk(clk), .reset(reset),
        .cpu_addr(cpu_addr), .cpu_wdata(cpu_wdata),
        .cpu_wstrb(cpu_wstrb), .cpu_we(cpu_we), .cpu_re(cpu_re),
        .cpu_rdata(cpu_rdata), .bus_error(bus_error),
        .irq_external(irq_external),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
        .i2c_scl_i(i2c_scl), .i2c_sda_i(i2c_sda),
        .i2c_scl_low(i2c_scl_low), .i2c_sda_low(i2c_sda_low),
        .pwm_out(pwm_out),
        .adc_start(adc_start), .adc_channel(adc_channel),
        .adc_busy(1'b0), .adc_done(1'b0), .adc_data(16'b0),
        .can_sel    (can_sel),
        .can_addr   (can_addr),
        .can_wdata  (can_wdata),
        .can_wstrb  (can_wstrb),
        .can_we     (can_we),
        .can_re     (can_re),
        .can_rdata  (can_rdata),
        .can_irq_rx (can_irq_rx),
        .can_irq_tx (can_irq_tx),
        .can_irq_err(can_irq_err)
    );

    // ------------------------------------------------------------
    // Person 1's CAN controller, attached through can_adapter --
    // the same wiring soc_top uses. periph_subsystem keeps its
    // slave port group; the controller hangs off it here.
    // ------------------------------------------------------------
    can_adapter #(
        .REGISTER_RDATA(1)
    ) U_CAN (
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
    // Bus tasks
    // ------------------------------------------------------------
    task bus_write;
        input [31:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            cpu_addr  = a;
            cpu_wdata = d;
            cpu_wstrb = 4'hF;
            cpu_we    = 1'b1;
            @(negedge clk);
            cpu_we    = 1'b0;
            cpu_wstrb = 4'h0;
        end
    endtask

    reg [31:0] rdval;

    task bus_read;
        input [31:0] a;
        begin
            @(negedge clk);
            cpu_addr = a;
            cpu_re   = 1'b1;
            @(negedge clk);
            rdval    = cpu_rdata;
            cpu_re   = 1'b0;
        end
    endtask

    task check;
        input [255:0] name;
        input [31:0]  got;
        input [31:0]  expect_val;
        begin
            if (got !== expect_val) begin
                $display("FAIL %0s : got %08h expected %08h",
                         name, got, expect_val);
                errors = errors + 1;
            end
            else begin
                $display("PASS %0s : %08h", name, got);
            end
        end
    endtask

    // ------------------------------------------------------------
    // Stimulus
    // ------------------------------------------------------------
    integer i;

    initial begin
        repeat (5) @(negedge clk);
        reset = 1'b0;
        repeat (2) @(negedge clk);

        // ========================================================
        $display("\n--- 1. Data memory through the decoder ---");
        bus_write(DMEM + 32'h10, 32'hDEAD_BEEF);
        bus_write(DMEM + 32'h14, 32'h1234_5678);
        bus_read (DMEM + 32'h10); check("dmem[0x10]", rdval, 32'hDEAD_BEEF);
        bus_read (DMEM + 32'h14); check("dmem[0x14]", rdval, 32'h1234_5678);

        // ========================================================
        $display("\n--- 2. Unmapped access raises bus_error ---");
        @(negedge clk);
        cpu_addr = 32'h9000_0000;
        cpu_re   = 1'b1;
        @(negedge clk);
        cpu_re   = 1'b0;
        if (bus_error !== 1'b1) begin
            $display("FAIL bus_error not asserted for 0x90000000");
            errors = errors + 1;
        end
        else $display("PASS bus_error asserted for unmapped region");

        // ========================================================
        $display("\n--- 3. GPIO ---");
        bus_write(GPIO + 32'h00, 32'h0000_00FF);   // low 8 = outputs
        bus_write(GPIO + 32'h04, 32'h0000_005A);   // drive 0x5A
        bus_read (GPIO + 32'h01*4);
        check("gpio OUT readback", rdval, 32'h0000_005A);
        if (gpio_out[7:0] !== 8'h5A) begin
            $display("FAIL gpio_out pins = %02h", gpio_out[7:0]);
            errors = errors + 1;
        end
        else $display("PASS gpio_out pins = 5A");

        // Rising-edge interrupt on pin 16
        bus_write(GPIO + 32'h0C, 32'h0001_0000);   // RISE_EN[16]
        bus_write(GPIO + 32'h18, 32'h0001_0000);   // IRQ_EN[16]
        gpio_drive[16] = 1'b0;
        repeat (4) @(negedge clk);
        gpio_drive[16] = 1'b1;                     // rising edge
        repeat (6) @(negedge clk);
        bus_read (GPIO + 32'h14);
        check("gpio IRQ_PEND", rdval, 32'h0001_0000);
        bus_write(GPIO + 32'h14, 32'h0001_0000);   // W1C
        bus_read (GPIO + 32'h14);
        check("gpio IRQ_PEND cleared", rdval, 32'h0000_0000);

        // ========================================================
        $display("\n--- 4. Timer ---");
        bus_write(TIMER + 32'h04, 32'd0);          // prescale 0
        bus_write(TIMER + 32'h08, 32'd20);         // compare 20
        bus_write(TIMER + 32'h10, 32'h1);          // clear status
        bus_write(TIMER + 32'h00, 32'b0111);       // en|reload|irq_en
        repeat (40) @(negedge clk);
        bus_read (TIMER + 32'h10);
        check("timer match flag", rdval & 32'h1, 32'h1);
        bus_write(TIMER + 32'h00, 32'b0000);       // stop
        bus_write(TIMER + 32'h10, 32'h1);

        // ========================================================
        $display("\n--- 5. PWM ---");
        bus_write(PWM + 32'h04, 32'd0);            // prescale
        bus_write(PWM + 32'h08, 32'd99);           // period 100 ticks
        bus_write(PWM + 32'h10, 32'd25);           // duty0 = 25%
        bus_write(PWM + 32'h20, 32'h1);            // enable ch0
        bus_write(PWM + 32'h00, 32'h1);            // enable
        repeat (250) @(negedge clk);
        $display("INFO pwm_out sampled = %b (scope the waveform)", pwm_out);

        // ========================================================
        $display("\n--- 6. UART loopback ---");
        bus_write(UART + 32'h08, 32'd2);           // fast osdiv for sim
        bus_write(UART + 32'h0C, 32'h1);           // rx irq enable
        bus_write(UART + 32'h00, 32'h0000_00A5);   // transmit 0xA5
        // 10 bit times * 16 oversamples * (osdiv+1) clocks + margin
        repeat (10*16*3 + 200) @(negedge clk);
        bus_read (UART + 32'h04);
        $display("INFO uart status = %08h", rdval);
        bus_read (UART + 32'h00);
        check("uart loopback byte", rdval & 32'hFF, 32'h0000_00A5);

        // ========================================================
        $display("\n--- 7. SPI loopback ---");
        bus_write(SPI + 32'h04, 32'h0000_0803);    // div 3, auto CS
        bus_write(SPI + 32'h00, 32'h0000_003C);    // send 0x3C
        repeat (200) @(negedge clk);
        bus_read (SPI + 32'h08);
        $display("INFO spi status = %08h", rdval);
        bus_read (SPI + 32'h00);
        check("spi loopback byte", rdval & 32'hFF, 32'h0000_003C);
        bus_write(SPI + 32'h08, 32'h2);            // clear done

        // ========================================================
        $display("\n--- 8. Interrupt controller ---");
        // Unmask GPIO (id 8) and TIMER (id 6)
        bus_write(INTC + 32'h04, 32'h1FF);         // clear all pending
        bus_write(INTC + 32'h00, (32'h1 << 8) | (32'h1 << 6));

        // Fire a GPIO edge
        bus_write(GPIO + 32'h14, 32'hFFFF_FFFF);
        gpio_drive[16] = 1'b0;
        repeat (4) @(negedge clk);
        gpio_drive[16] = 1'b1;
        repeat (8) @(negedge clk);

        bus_read (INTC + 32'h04);
        $display("INFO intc pending = %08h", rdval);
        bus_read (INTC + 32'h08);
        check("intc cause = GPIO (8)", rdval, 32'd8);
        if (irq_external !== 1'b1) begin
            $display("FAIL irq_external not asserted");
            errors = errors + 1;
        end
        else $display("PASS irq_external asserted");

        // Clear it and confirm the line drops
        bus_write(GPIO + 32'h14, 32'hFFFF_FFFF);
        bus_write(INTC + 32'h04, 32'h1FF);
        repeat (4) @(negedge clk);
        if (irq_external !== 1'b0) begin
            $display("FAIL irq_external stuck high");
            errors = errors + 1;
        end
        else $display("PASS irq_external cleared");

        // ========================================================
        $display("\n--- 9. CAN register access through the bus ---");
        //
        // This is the check neither team's own testbench can make.
        //
        // can_regfile.v drives rdata from an always @(*) block, so
        // read data is valid in the SAME cycle as the request.
        // Person 1's tb_can_controller_top samples it that way too
        // (`#1; rd_result = rdata;`), so it structurally cannot see
        // the mismatch. This bus requires data one cycle LATER --
        // bus_interconnect.v selects each slave via sel_q.
        //
        // can_adapter.v bridges the two by registering rdata at the
        // boundary. If that conversion is wrong, the readback below
        // returns a stale or wrong word and this fails loudly here,
        // rather than as garbage on the board.
        // ========================================================
        bus_write(CAN + CAN_CTRL, 32'b011);        // EN=1, LOOPBACK=1
        bus_read (CAN + CAN_CTRL);
        check("can CTRL readback", rdval, 32'h0000_0003);

        // Same bit timing Person 1's own self-test uses:
        // BRP=0, PROP_SEG=2, PHASE_SEG1=2, PHASE_SEG2=2, SJW=1
        bus_write(CAN + CAN_BTIME, {10'd0, 2'd1, 4'd2, 4'd2, 4'd2, 8'd0});
        bus_read (CAN + CAN_BTIME);
        check("can BTIME readback", rdval,
              {10'd0, 2'd1, 4'd2, 4'd2, 4'd2, 8'd0});

        // Two different values back to back. A boundary shim that
        // is off by a cycle passes a single read by luck but shows
        // the previous word here.
        bus_write(CAN + CAN_IE, 32'h0000_0007);
        bus_read (CAN + CAN_IE);
        check("can IE = 7", rdval, 32'h0000_0007);
        bus_write(CAN + CAN_IE, 32'h0000_0001);
        bus_read (CAN + CAN_IE);
        check("can IE = 1 (no stale data)", rdval, 32'h0000_0001);

        // STATUS after reset: both FIFOs empty, error-active.
        bus_read (CAN + CAN_STATUS);
        $display("INFO can STATUS = %08h", rdval);
        check("can TX FIFO empty", rdval & 32'h2, 32'h2);
        check("can RX FIFO empty", rdval & 32'h8, 32'h8);

        // ========================================================
        $display("\n--- 10. CAN loopback frame and interrupt ---");
        // ========================================================
        bus_write(CAN + CAN_IE, 32'h0000_0007);    // all three sources
        bus_write(INTC + 32'h04, 32'h1FF);         // clear pending
        bus_write(INTC + 32'h00, 32'h007);         // unmask IRQ 0,1,2

        bus_write(CAN + CAN_TX_ID,    32'h0000_0245);
        bus_write(CAN + CAN_TX_DATA0, 32'hDEAD_BEEF);
        bus_write(CAN + CAN_TX_DATA1, 32'h1234_5678);
        bus_write(CAN + CAN_TX_CTRL,  {26'd0, 1'b0, 1'b0, 4'd8});

        // A full 8-byte frame at 7 sysclk per bit is a few thousand
        // cycles. Poll rather than guess a fixed delay.
        // Read once BEFORE testing, otherwise the loop exits on the
        // initial value of rdval without ever polling the hardware.
        can_timeout = 0;
        bus_read(CAN + CAN_STATUS);
        while ((rdval[3] === 1'b1) && (can_timeout < 60000)) begin
            bus_read(CAN + CAN_STATUS);
            can_timeout = can_timeout + 1;
        end

        if (can_timeout >= 60000) begin
            $display("FAIL can loopback frame never arrived");
            errors = errors + 1;
        end
        else begin
            $display("PASS can RX FIFO non-empty after %0d polls",
                     can_timeout);

            bus_read(CAN + CAN_RX_ID);
            check("can RX ID", rdval, 32'h0000_0245);
            bus_read(CAN + CAN_RX_DATA0);
            check("can RX DATA0", rdval, 32'hDEAD_BEEF);
            bus_read(CAN + CAN_RX_DATA1);
            check("can RX DATA1", rdval, 32'h1234_5678);

            // RX_CTRL read also pops the FIFO. If can_adapter.v
            // captured the post-pop word instead of the pre-pop
            // head, the DLC here is wrong.
            bus_read(CAN + CAN_RX_CTRL);
            check("can RX DLC = 8 (pop-on-read)", rdval & 32'hF, 32'd8);

            // The interrupt should have reached the INTC. CAN RX is
            // IRQ 0, the highest priority source in the system.
            bus_read(INTC + 32'h04);
            $display("INFO intc pending = %08h", rdval);
            check("intc pending has CAN RX", rdval & 32'h1, 32'h1);
            bus_read(INTC + 32'h08);
            check("intc cause = CAN RX (0)", rdval, 32'd0);

            if (irq_external !== 1'b1) begin
                $display("FAIL irq_external not asserted for CAN");
                errors = errors + 1;
            end
            else $display("PASS irq_external asserted for CAN");

            // Clear at the source first, then at the INTC.
            bus_write(CAN + CAN_IP, 32'h0000_0007);
            bus_read (CAN + CAN_IP);
            check("can IP cleared", rdval & 32'h7, 32'h0);
            bus_write(INTC + 32'h04, 32'h1FF);
            repeat (4) @(negedge clk);
            if (irq_external !== 1'b0) begin
                $display("FAIL CAN irq stuck high after clear");
                errors = errors + 1;
            end
            else $display("PASS CAN irq cleared cleanly");
        end

        // ========================================================
        $display("\n========================================");
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("%0d CHECK(S) FAILED", errors);
        $display("========================================\n");
        $finish;
    end

    initial begin
        // Raised for the CAN loopback frame: a full 8-byte frame at
        // 7 sysclk per time quantum takes several thousand cycles.
        #40_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
