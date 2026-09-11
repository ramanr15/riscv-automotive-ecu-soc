# CAN Controller Register Map (Person 1 deliverable)

Per WBS Section 2.3.10 ("Register Map — Deliver This to Person 2 and Person 3")
and the Documentation Standard in Section 1.4.4. This file is the
living register-map document; it grows as each RTL step lands.

Base address on the SoC bus (Section 1.4.2): **`0x2000_0000`**, assigned to
Person 1 by the shared integration contract.

## Bus convention reminder (Section 1.4.1)

`clk, rst_n, addr[31:0], wdata[31:0], rdata[31:0], we, re, sel, ready` —
32-bit address and data, byte-addressable registers accessed as 32-bit
words, unused bits read as zero. **Note:** the base RV32IM core
(`top_level_riscv.v`) currently uses an active-*high* `reset`, while this
SoC-level contract specifies active-*low* `rst_n`. Person 2's bus/top-level
SoC wrapper needs one inverter (or a small reset synchronizer) between the
CPU's `reset` and the peripheral-side `rst_n` net. Every module in this CAN
controller uses `rst_n` (active-low) internally to match the contract.

## CAN_CTRL — offset 0x00, R/W

| Bits | Field | Description |
|---|---|---|
| 0 | EN | Controller enable (gates the bit timing generator and all sequencing) |
| 1 | LOOPBACK | Internal loopback test mode (TX looped to RX inside the controller, per Section 2.5's self-test) |
| 2 | RESET | Soft reset of frame-level state (does not affect TEC/REC) |
| 31:3 | — | Reserved, read as 0 |

## CAN_BTIME — offset 0x04, R/W

Bit timing configuration consumed by `can_bit_timing.v` (Step 1). This
exact layout was not fixed by the WBS text, so it is defined here as
the authoritative contract for Person 2 (bus wiring) and Person 3
(verification):

| Bits | Field | Description |
|---|---|---|
| 7:0 | BRP | Baud rate prescaler; TQ length = (BRP+1) system clocks |
| 11:8 | PROP_SEG | Propagation segment length, in TQ (0 treated as 1) |
| 15:12 | PHASE_SEG1 | Phase segment 1 length, in TQ (0 treated as 1) |
| 19:16 | PHASE_SEG2 | Phase segment 2 length, in TQ (0 treated as 1) |
| 21:20 | SJW | Resynchronization jump width, in TQ (0 treated as 1) |
| 31:22 | — | Reserved, read as 0 |

Total bit length = 1 (fixed SYNC_SEG) + PROP_SEG + PHASE_SEG1 + PHASE_SEG2 TQ.
`sample_point` (the moment the bus is latched) falls at the PHASE_SEG1 /
PHASE_SEG2 boundary, i.e. `1 + PROP_SEG + PHASE_SEG1` time quanta after
Start-of-Frame.

## CAN_STATUS — offset 0x08, R  *(bit assignment to be finalized in Step 9)*

Planned per Section 2.3.9: TX FIFO full/empty, RX FIFO full/empty,
2-bit error state (Error-Active/Passive/Bus-Off, see `can_defs.v`
`CAN_ERR_*`).

## CAN_TX_ID / CAN_TX_DATA0 / CAN_TX_DATA1 / CAN_TX_CTRL — 0x0C/0x10/0x14/0x18

Feed `can_frame_fifo.v` (TX instance). Field widths match the FIFO
entry: `id[28:0]`, `ide`, `rtr`, `dlc[3:0]`, `data[63:0]`. Exact
per-register bit packing to be finalized alongside Step 9.

## CAN_RX_ID / CAN_RX_DATA0 / CAN_RX_DATA1 / CAN_RX_CTRL — 0x1C/0x20/0x24/0x28

Read from `can_frame_fifo.v` (RX instance). Per Section 2.4.2, reading
`CAN_RX_CTRL` is the pop strobe and must be read **last**, after
`CAN_RX_ID`/`CAN_RX_DATA0`/`CAN_RX_DATA1` — the FIFO exposes the head
entry continuously (combinational read), so this ordering is safe.

## CAN_IE / CAN_IP — offset 0x2C (R/W) / 0x30 (R/W1C)

Interrupt enable mask / pending flags. Bit positions match
`can_defs.v`: `CAN_INT_RX_BIT=0`, `CAN_INT_TX_BIT=1`, `CAN_INT_ERR_BIT=2`
— which also line up with the SoC-level IRQ ID map Person 2 wires into
the interrupt controller (Section 1.4.3: CAN RX=0, CAN TX=1, CAN
Error=2).

## CAN_TEC_REC — offset 0x34, R

8-bit Transmit Error Counter and 8-bit Receive Error Counter from the
Step 8 error state machine (`can_defs.v` thresholds: passive at 128,
bus-off at 255).

---

*Status: CAN_CTRL/CAN_BTIME finalized (backing RTL: `can_bit_timing.v`).
Remaining registers will be finalized together with Step 9
(`can_controller_top.v`) once the register file is written — this
document will be updated in place rather than superseded.*
