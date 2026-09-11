`timescale 1ns / 1ps
// ============================================================
// csr_unit  -- MODIFIED BY PERSON 2
//
// Added for the peripheral subsystem:
//   * mie  (0x304)  machine interrupt enable
//   * mip  (0x344)  machine interrupt pending (MEIP is read-only,
//                   it simply follows the interrupt controller)
//   * irq_external  input from intc
//   * interrupt_request output, asserted when
//        mstatus.MIE & mie.MEIE & mip.MEIP
//
// Everything else is unchanged from the original file.
// ============================================================
module csr_unit (

    input wire clk,
    input wire reset,

    // CSR instruction access
    input wire csr_enable,

    input wire [2:0] csr_funct3,

    input wire [11:0] csr_address,

    input wire [31:0] rs1_value,
    input wire [4:0] zimm,

    output reg [31:0] csr_read_data,


    // Trap entry
    input wire trap_enter,

    input wire [31:0] trap_pc,
    input wire [31:0] trap_cause,
    input wire [31:0] trap_value,


    // MRET
    input wire mret,

    output wire [31:0] mtvec_out,
    output wire [31:0] mepc_out,

    // ---------------- NEW: external interrupt ----------------
    input  wire irq_external,
    output wire interrupt_request

);


// ============================================================
// MACHINE CSRs
// ============================================================

reg [31:0] mstatus;
reg [31:0] mtvec;

reg [31:0] mepc;
reg [31:0] mcause;
reg [31:0] mtval;

reg [31:0] mie;                 // NEW

reg [63:0] mcycle;
reg [63:0] minstret;


assign mtvec_out = mtvec;

assign mepc_out = mepc;


// ============================================================
// mip : only MEIP (bit 11) is implemented, and it is a direct
// read-only reflection of the interrupt controller output.
// ============================================================

wire [31:0] mip;

assign mip = {20'b0, irq_external, 11'b0};


// ============================================================
// Interrupt request into the pipeline
//   mstatus.MIE = bit 3
//   mie.MEIE    = bit 11
//   mip.MEIP    = bit 11
// ============================================================

assign interrupt_request = mstatus[3] & mie[11] & mip[11];


// ============================================================
// CSR READ
// ============================================================

always @(*) begin

    case (csr_address)

        12'h300:
            csr_read_data = mstatus;

        12'h304:
            csr_read_data = mie;

        12'h305:
            csr_read_data = mtvec;

        12'h341:
            csr_read_data = mepc;

        12'h342:
            csr_read_data = mcause;

        12'h343:
            csr_read_data = mtval;

        12'h344:
            csr_read_data = mip;

        12'hB00:
            csr_read_data = mcycle[31:0];

        12'hB80:
            csr_read_data = mcycle[63:32];

        12'hB02:
            csr_read_data = minstret[31:0];

        12'hB82:
            csr_read_data = minstret[63:32];

        default:
            csr_read_data = 32'b0;

    endcase

end


// ============================================================
// CSR SOURCE
// ============================================================

reg [31:0] csr_source;

always @(*) begin

    if (csr_funct3[2])
        csr_source = {27'b0, zimm};
    else
        csr_source = rs1_value;

end


// ============================================================
// CSR WRITE VALUE
// ============================================================

reg [31:0] csr_write_value;

always @(*) begin

    case (csr_funct3[1:0])

        // CSRRW / CSRRWI
        2'b01:
            csr_write_value = csr_source;

        // CSRRS / CSRRSI
        2'b10:
            csr_write_value =
                csr_read_data | csr_source;

        // CSRRC / CSRRCI
        2'b11:
            csr_write_value =
                csr_read_data & ~csr_source;

        default:
            csr_write_value = csr_read_data;

    endcase

end


// ============================================================
// CSR UPDATE
// ============================================================

always @(posedge clk) begin

    if (reset) begin

        mstatus <= 32'b0;

        // Trap vector
        mtvec <= 32'h00000100;

        mepc   <= 32'b0;
        mcause <= 32'b0;
        mtval  <= 32'b0;

        mie    <= 32'b0;

        mcycle   <= 64'b0;
        minstret <= 64'b0;

    end

    else begin

        mcycle <= mcycle + 64'd1;


        // --------------------------------------------------------
        // TRAP ENTRY (exception or interrupt)
        // --------------------------------------------------------

        if (trap_enter) begin

            mepc   <= trap_pc;

            mcause <= trap_cause;

            mtval  <= trap_value;

            // MIE -> MPIE
            mstatus[7] <= mstatus[3];

            // Disable interrupts
            mstatus[3] <= 1'b0;

            // MPP = Machine
            mstatus[12:11] <= 2'b11;

        end


        // --------------------------------------------------------
        // MRET
        // --------------------------------------------------------

        else if (mret) begin

            mstatus[3] <= mstatus[7];

            mstatus[7] <= 1'b1;

            mstatus[12:11] <= 2'b00;

        end


        // --------------------------------------------------------
        // CSR WRITE
        // --------------------------------------------------------

        else if (csr_enable) begin

            case (csr_address)

                12'h300:
                    mstatus <= csr_write_value;

                12'h304:
                    mie <= csr_write_value;

                12'h305:
                    mtvec <= csr_write_value;

                12'h341:
                    mepc <= csr_write_value;

                12'h342:
                    mcause <= csr_write_value;

                12'h343:
                    mtval <= csr_write_value;

                // 12'h344 mip : MEIP is read-only, writes ignored

                default: begin
                end

            endcase

        end

    end

end

endmodule
