// ============================================================
// can_defs.v
//
// Shared parameters / macros for the CAN Controller RTL owned by
// Person 1 (Rama Krishna Prasadh H) in the RISC-V Automotive ECU
// SoC project.
//
// Style follows the base RV32IM core's Def.v (plain Verilog-2001
// `define guards, no SystemVerilog), so this drops into the same
// toolflow (Vivado/XSim) that Raman R's base core already uses.
// ============================================================

`ifndef CAN_DEF_V
`define CAN_DEF_V

// ------------------------------------------------------------
// Bus level encoding (matches the physical CAN bus convention)
// ------------------------------------------------------------
`define CAN_RECESSIVE   1'b1
`define CAN_DOMINANT    1'b0

// ------------------------------------------------------------
// Frame field state machine states (Section 2.3.6 of the WBS)
// One-hot is avoided to keep the state register small; binary
// encoding is fine at this width and this is not on a timing
// critical path (one transition per bit time at most).
// ------------------------------------------------------------
`define CAN_FLD_IDLE          4'd0
`define CAN_FLD_SOF           4'd1
`define CAN_FLD_ARBITRATION   4'd2
`define CAN_FLD_CONTROL       4'd3
`define CAN_FLD_DATA          4'd4
`define CAN_FLD_CRC           4'd5
`define CAN_FLD_CRC_DELIM     4'd6
`define CAN_FLD_ACK_SLOT      4'd7
`define CAN_FLD_ACK_DELIM     4'd8
`define CAN_FLD_EOF           4'd9
`define CAN_FLD_INTERMISSION  4'd10
`define CAN_FLD_ERROR         4'd11
`define CAN_FLD_OVERLOAD      4'd12

// ------------------------------------------------------------
// Error state machine states (Section 2.3.9 / error confinement)
// ------------------------------------------------------------
`define CAN_ERR_ACTIVE   2'b00
`define CAN_ERR_PASSIVE  2'b01
`define CAN_ERR_BUSOFF   2'b10

// ------------------------------------------------------------
// Error counter thresholds (ISO 11898-1)
// ------------------------------------------------------------
`define CAN_TEC_PASSIVE_THRESH   9'd128
`define CAN_REC_PASSIVE_THRESH   9'd128
`define CAN_TEC_BUSOFF_THRESH    9'd255
`define CAN_BUSOFF_RECOVERY_CNT  8'd128   // 128 x 11 consecutive recessive bits

// ------------------------------------------------------------
// Frame geometry
// ------------------------------------------------------------
`define CAN_ID_STD_WIDTH   11
`define CAN_ID_EXT_WIDTH   29
`define CAN_CRC_WIDTH      15
`define CAN_MAX_DLC        4'd8   // bytes
`define CAN_FIFO_DEPTH     8      // frames, per Section 2.3.3

// ------------------------------------------------------------
// Interrupt bit positions inside CAN_IE / CAN_IP (Section 2.3.10)
// These also line up with the SoC-level IRQ ID map in Section 1.4.3
// (CAN RX=0, CAN TX=1, CAN Error=2) that Person 2 wires into the
// interrupt controller.
// ------------------------------------------------------------
`define CAN_INT_RX_BIT     0
`define CAN_INT_TX_BIT     1
`define CAN_INT_ERR_BIT    2

`endif // CAN_DEF_V
