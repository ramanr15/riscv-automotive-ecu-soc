`timescale 1ns / 1ps
// ============================================================
// tb_top_level_riscv  -- UPDATED FOR THE PERIPHERAL SUBSYSTEM
//
// Same testbench you already had. Every monitor and every
// DUT.<signal> path is unchanged, so your existing pipeline
// table still works exactly as before.
//
// WHAT CHANGED
//   1. The DUT instantiation now connects the new ports:
//      irq_external, dmem_* and debug_irq_taken.
//   2. A periph_subsystem is attached to the CPU's memory port.
//      This provides the data memory (which now lives outside
//      the CPU) plus every peripheral, so loads and stores work
//      and software can reach 0x4000_0000.
//   3. New interrupt monitor and a TRAP/IRQ column.
// ============================================================

module tb_top_level_riscv;

    reg clk;
    reg reset;

    integer cycle;
    integer i;

// ============================================================
// DEBUG OUTPUT WIRES FROM PROCESSOR
// ============================================================

wire [31:0] debug_pc;
wire [31:0] debug_instruction;
wire [31:0] debug_alu_result;
wire [31:0] debug_wb_data;
wire [4:0]  debug_wb_rd;
wire        debug_reg_write;
wire        debug_stall;
wire        debug_branch_taken;
wire        debug_irq_taken;

// ============================================================
// CPU <-> PERIPHERAL SUBSYSTEM
// ============================================================

wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire [3:0]  dmem_wstrb;
wire        dmem_we;
wire        dmem_re;
wire [31:0] dmem_rdata;

wire        irq_external;
wire        bus_error;

// ============================================================
// PERIPHERAL PADS
// ============================================================

reg  [31:0] gpio_drive = 32'b0;
wire [31:0] gpio_out;
wire [31:0] gpio_oe;
wire [31:0] gpio_in;

// The subsystem drives pins configured as outputs; the
// testbench supplies the rest.
assign gpio_in = (gpio_out & gpio_oe) | (gpio_drive & ~gpio_oe);

wire uart_tx;
wire uart_rx;
assign uart_rx = uart_tx;                 // loopback

wire        spi_sclk;
wire        spi_mosi;
wire [3:0]  spi_cs_n;
wire        spi_miso;
assign spi_miso = spi_mosi;               // loopback

wire i2c_scl_low, i2c_sda_low;
wire i2c_scl = i2c_scl_low ? 1'b0 : 1'b1; // pull-ups, no slave present
wire i2c_sda = i2c_sda_low ? 1'b0 : 1'b1;

wire [3:0] pwm_out;

wire       adc_start;
wire [3:0] adc_channel;

    // ============================================================
    // DUT
    // ============================================================

top_level_riscv DUT (
    .clk                (clk),
    .reset              (reset),

    .irq_external       (irq_external),

    .dmem_addr          (dmem_addr),
    .dmem_wdata         (dmem_wdata),
    .dmem_wstrb         (dmem_wstrb),
    .dmem_we            (dmem_we),
    .dmem_re            (dmem_re),
    .dmem_rdata         (dmem_rdata),

    .debug_pc            (debug_pc),
    .debug_instruction   (debug_instruction),
    .debug_alu_result    (debug_alu_result),
    .debug_wb_data       (debug_wb_data),
    .debug_wb_rd         (debug_wb_rd),
    .debug_reg_write     (debug_reg_write),
    .debug_stall         (debug_stall),
    .debug_branch_taken  (debug_branch_taken),
    .debug_irq_taken     (debug_irq_taken)
);

    // ============================================================
    // DATA MEMORY + PERIPHERALS
    // ============================================================

periph_subsystem #(
    .GPIO_WIDTH  (32),
    .PWM_CHANNELS(4),
    .SPI_CS      (4)
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

    .i2c_scl_i   (i2c_scl),
    .i2c_sda_i   (i2c_sda),
    .i2c_scl_low (i2c_scl_low),
    .i2c_sda_low (i2c_sda_low),

    .pwm_out     (pwm_out),

    .adc_start   (adc_start),
    .adc_channel (adc_channel),
    .adc_busy    (1'b0),
    .adc_done    (1'b0),
    .adc_data    (16'b0),

    // Person 1's CAN controller is not present yet.
    .can_sel     (),
    .can_addr    (),
    .can_wdata   (),
    .can_wstrb   (),
    .can_we      (),
    .can_re      (),
    .can_rdata   (32'b0),
    .can_irq_rx  (1'b0),
    .can_irq_tx  (1'b0),
    .can_irq_err (1'b0)
);

    // ============================================================
    // CLOCK
    // ============================================================

    initial begin

        clk = 1'b0;

        forever #5 clk = ~clk;

    end

    // ============================================================
    // INITIALIZATION
    // ============================================================

    initial begin

        cycle = 0;

        reset = 1'b1;

        // Reset for 2 clock periods
        #20;

        reset = 1'b0;

        // Run for 400 cycles
        #4000;

        // ========================================================
        // FINAL REGISTER DUMP
        // ========================================================

        $display("");
        $display("");
        $display("================================================================================");
        $display("                         FINAL RV32IM REGISTER DUMP");
        $display("================================================================================");
        $display(" REGISTER       HEX VALUE        SIGNED DECIMAL       UNSIGNED DECIMAL");
        $display("--------------------------------------------------------------------------------");

        for (i = 0; i < 32; i = i + 1) begin

            $display(
                " x%02d           %08h         %12d         %12d",
                i,
                DUT.REGISTER_FILE.registers[i],
                $signed(DUT.REGISTER_FILE.registers[i]),
                DUT.REGISTER_FILE.registers[i]
            );

        end

        $display("================================================================================");

        $display("");
        $display("FINAL PC       = 0x%08h", DUT.pc);

        $display("MTVEC          = 0x%08h", DUT.CSR_UNIT.mtvec);
        $display("MEPC           = 0x%08h", DUT.CSR_UNIT.mepc);
        $display("MCAUSE         = 0x%08h", DUT.CSR_UNIT.mcause);
        $display("MTVAL          = 0x%08h", DUT.CSR_UNIT.mtval);
        $display("MSTATUS        = 0x%08h", DUT.CSR_UNIT.mstatus);
        $display("MIE            = 0x%08h", DUT.CSR_UNIT.mie);
        $display("MIP            = 0x%08h", DUT.CSR_UNIT.mip);

        $display("");
        $display("--------------------------------------------------------------------------------");
        $display("                        PERIPHERAL FINAL STATE");
        $display("--------------------------------------------------------------------------------");
        $display("GPIO DIR       = 0x%08h", PERIPHERALS.GPIO.dir);
        $display("GPIO OUT       = 0x%08h", PERIPHERALS.GPIO.out_val);
        $display("GPIO IN        = 0x%08h", PERIPHERALS.GPIO.sync1);
        $display("GPIO IRQ_PEND  = 0x%08h", PERIPHERALS.GPIO.irq_pend);
        $display("TIMER COUNT    = 0x%08h", PERIPHERALS.TIMER.CORE.count);
        $display("TIMER STATUS   = %b",     PERIPHERALS.TIMER.match_flag);
        $display("INTC MASK      = 0x%08h", PERIPHERALS.INTC.mask);
        $display("INTC PENDING   = 0x%08h", PERIPHERALS.INTC.pending);
        $display("INTC CAUSE     = %0d",    PERIPHERALS.INTC.cause);
        $display("IRQ TO CPU     = %b",     irq_external);

        $display("");
        $display("================================================================================");
        $display("                           SIMULATION COMPLETED");
        $display("================================================================================");
        $display("");

        $finish;

    end

    // ============================================================
    // CYCLE COUNTER
    // ============================================================

    always @(posedge clk) begin

        if (reset)

            cycle <= 0;

        else

            cycle <= cycle + 1;

    end

    // ============================================================
    // PIPELINE TABLE
    // ============================================================

    initial begin

        $display("");
        $display("==========================================================================================================================================================================================");
        $display("                                                   RV32IM 5-STAGE PIPELINE EXECUTION TABLE");
        $display("==========================================================================================================================================================================================");

        $display(
        "CYCLE | IF_PC    | IF_INSTR | ID_PC    | ID_INSTR | EX_PC    | EX_INSTR | EX_RESULT | MEM_RESULT | WB_RD | WB_RESULT | STALL | FWD_A | FWD_B | BR | PRED | REDIR | EXC | IRQ"
        );

        $display(
        "------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"
        );

    end

    always @(posedge clk) begin

        if (!reset) begin

            #1;

            $display(
                "%5d | %08h | %08h | %08h | %08h | %08h | %08h | %08h | %08h   | x%02d  | %08h  |   %b   |  %02b   |  %02b   | %b  |  %b   |   %b   |  %b  |  %b",

                cycle,

                DUT.pc,
                DUT.instruction_if,

                DUT.pc_id,
                DUT.instruction_id,

                DUT.pc_ex,
                DUT.instruction_ex,

                DUT.execute_result_ex,

                DUT.execute_result_mem,

                DUT.rd_wb,

                DUT.wb_result,

                DUT.stall,

                DUT.forward_a,

                DUT.forward_b,

                DUT.actual_branch_taken_ex,

                DUT.predicted_taken_ex,

                DUT.redirect_ex,

                DUT.exception_ex,

                DUT.irq_take
            );

        end

    end

    // ============================================================
    // WRITEBACK MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset &&
            DUT.reg_write_wb &&
            (DUT.rd_wb != 0)) begin

            #1;

            $display(
                "      >>> WRITEBACK : x%0d <= 0x%08h   signed=%0d",
                DUT.rd_wb,
                DUT.wb_result,
                $signed(DUT.wb_result)
            );

        end

    end

    // ============================================================
    // STALL MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.stall) begin

            $display("");
            $display(
                "      >>> LOAD-USE STALL detected at ID PC = 0x%08h",
                DUT.pc_id
            );
            $display("");

        end

    end

    // ============================================================
    // FORWARDING MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset) begin

            if (DUT.forward_a != 2'b00)

                $display(
                    "      >>> FORWARD A : selector = %02b",
                    DUT.forward_a
                );

            if (DUT.forward_b != 2'b00)

                $display(
                    "      >>> FORWARD B : selector = %02b",
                    DUT.forward_b
                );

        end

    end

    // ============================================================
    // RV32M MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.is_muldiv_ex) begin

            $display("");
            $display(
                "      >>> RV32M OPERATION : PC=%08h FUNCT3=%03b A=%08h B=%08h RESULT=%08h",
                DUT.pc_ex,
                DUT.funct3_ex,
                DUT.forwarded_a,
                DUT.forwarded_b,
                DUT.muldiv_result_ex
            );
            $display("");

        end

    end

    // ============================================================
    // BRANCH MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.branch_ex) begin

            $display("");
            $display(
                "      >>> BRANCH : PC=%08h ACTUAL=%b PREDICTED=%b TARGET=%08h",
                DUT.pc_ex,
                DUT.actual_branch_taken_ex,
                DUT.predicted_taken_ex,
                DUT.branch_target_ex
            );

            if (DUT.branch_mispredict_ex)

                $display(
                    "      >>> BRANCH MISPREDICTION -> RECOVERY PC = %08h",
                    DUT.recovery_pc_ex
                );

            $display("");

        end

    end

    // ============================================================
    // JUMP MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.jump_ex) begin

            $display("");
            $display(
                "      >>> JUMP : PC=%08h TARGET=%08h",
                DUT.pc_ex,
                DUT.jump_target_ex
            );
            $display("");

        end

    end

// ============================================================
// MEMORY ACCESS MONITOR
// Now also decodes which slave the access was routed to.
// ============================================================

// Helper for readable region names in the monitor output.
function [95:0] region_name;
    input [31:0] a;
    begin
        if (a[31:28] == 4'h0)
            region_name = "DATA MEM   ";
        else if (a[31:28] == 4'h4) begin
            case (a[19:16])
                4'd0: region_name = "GPIO       ";
                4'd1: region_name = "TIMER      ";
                4'd2: region_name = "PWM        ";
                4'd3: region_name = "UART       ";
                4'd4: region_name = "SPI        ";
                4'd5: region_name = "I2C        ";
                4'd6: region_name = "ADC        ";
                4'd7: region_name = "CAN        ";
                4'd8: region_name = "INTC       ";
                default: region_name = "UNMAPPED   ";
            endcase
        end
        else
            region_name = "UNMAPPED   ";
    end
endfunction

always @(negedge clk) begin

    // --------------------------------------------------------
    // STORE
    // --------------------------------------------------------
    if (!reset && DUT.mem_write_mem_reg) begin

        $display("");

        $display(
            "      >>> MEMORY WRITE : ADDR=%08h DATA=%08h MASK=%04b  [%0s]",
            DUT.execute_result_mem,
            DUT.formatted_store_data,
            DUT.write_mask,
            region_name(DUT.execute_result_mem)
        );

        $display("");

    end

    // --------------------------------------------------------
    // LOAD REQUEST
    // --------------------------------------------------------

    if (!reset && DUT.mem_read_mem_reg) begin

        $display("");

        $display(
            "      >>> MEMORY READ REQUEST : ADDR=%08h  [%0s]",
            DUT.execute_result_mem,
            region_name(DUT.execute_result_mem)
        );

        $display("");

    end


    // --------------------------------------------------------
    // LOAD RESPONSE
    // --------------------------------------------------------

    if (!reset && DUT.mem_to_reg_delay) begin

        $display("");

        $display(
            "      >>> MEMORY READ RESPONSE : ADDR=%08h RAW=%08h FORMATTED=%08h",
            DUT.load_address_delay,
            DUT.memory_read_data,
            DUT.formatted_load_data
        );

        $display("");

    end

end

    // ============================================================
    // BUS ERROR MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && bus_error) begin

            $display("");
            $display("      !!! BUS ERROR : access to an unmapped address");
            $display("");

        end

    end

    // ============================================================
    // CSR MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.csr_instruction_ex) begin

            $display("");
            $display(
                "      >>> CSR OPERATION : PC=%08h CSR=%03h OLD_VALUE=%08h",
                DUT.pc_ex,
                DUT.instruction_ex[31:20],
                DUT.csr_read_data_ex
            );
            $display("");

        end

    end

    // ============================================================
    // EXCEPTION / TRAP MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.exception_ex) begin

            $display("");
            $display("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            $display("                     EXCEPTION / TRAP");
            $display("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

            $display("PC       = 0x%08h", DUT.pc_ex);
            $display("INSTR    = 0x%08h", DUT.instruction_ex);
            $display("CAUSE    = %0d",    DUT.exception_cause_ex);
            $display("MTVAL    = 0x%08h", DUT.exception_value_ex);
            $display("MTVEC    = 0x%08h", DUT.mtvec);
            $display("REDIRECT = 0x%08h", DUT.mtvec);

            $display("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            $display("");

        end

    end

    // ============================================================
    // EXTERNAL INTERRUPT MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.irq_take) begin

            $display("");
            $display("############################################################");
            $display("                  EXTERNAL INTERRUPT TAKEN");
            $display("############################################################");

            $display("INTERRUPTED PC = 0x%08h", DUT.pc_ex);
            $display("INTC CAUSE     = %0d  (IRQ ID)", PERIPHERALS.INTC.cause);
            $display("INTC PENDING   = 0x%08h", PERIPHERALS.INTC.pending);
            $display("MCAUSE WILL BE = 0x8000000B");
            $display("VECTOR         = 0x%08h", DUT.mtvec);

            $display("############################################################");
            $display("");

        end

    end

    // ============================================================
    // MRET MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.mret_ex) begin

            $display("");
            $display(
                "      >>> MRET : Returning to MEPC = 0x%08h",
                DUT.mepc
            );
            $display("");

        end

    end

    // ============================================================
    // FENCE.I MONITOR
    // ============================================================

    always @(posedge clk) begin

        if (!reset && DUT.fence_i_ex) begin

            $display("");
            $display(
                "      >>> FENCE.I : Fetch pipeline flushed"
            );
            $display("");

        end

    end

endmodule
