# CAN Controller Integration Spec

**For:** Person 1 (CAN Controller)
**From:** Person 2 (Bus / Interrupt Controller / Peripherals)

The peripheral subsystem is finished and verified. A slave port
and three interrupt lines are already wired and waiting for the
CAN controller. Nothing inside the subsystem needs to change
when it arrives — integration is a few lines in `soc_top.v`.

This document is everything needed to build against it.

---

## 1. Module interface

The CAN controller must present exactly these ports. Names can
differ; widths and behaviour cannot.

```verilog
module can_controller (
    input  wire        clk,
    input  wire        reset,      // synchronous, active high

    // ---- bus slave port ----
    input  wire        sel,        // this peripheral is addressed
    input  wire [31:0] addr,       // full address; use addr[7:2]
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,      // always 4'hF in practice
    input  wire        we,
    input  wire        re,
    output reg  [31:0] prdata,     // REGISTERED - see rule 1

    // ---- interrupts ----
    output wire        irq_rx,
    output wire        irq_tx,
    output wire        irq_err,

    // ---- CAN pads ----
    input  wire        can_rx,
    output wire        can_tx
);
```

---

## 2. Three rules

### Rule 1 — `prdata` must be registered

Request on cycle N, data available on cycle N+1. This is the
same contract the data memory uses, and the CPU's MEM stage
already delays its load metadata by one cycle to match.

```verilog
wire wr = sel & we;
wire rd = sel & re;
wire [5:0] reg_index = addr[7:2];

always @(posedge clk) begin
    if (reset)
        prdata <= 32'b0;
    else if (rd) begin
        case (reg_index)
            6'h00:   prdata <= mode_reg;
            6'h01:   prdata <= status_reg;
            // ...
            default: prdata <= 32'b0;
        endcase
    end
end
```

**A combinational read will produce corrupt data.** There is no
`ready` signal and the CPU has no memory-stall path — it cannot
wait. If a read takes longer than one cycle, the CPU will latch
whatever happens to be on the bus.

If some operation genuinely needs many cycles, do what SPI and
I2C do: start it from a register write, expose a `busy` bit, and
raise an interrupt on completion. The register access itself
still completes in one cycle.

### Rule 2 — Base address `0x4007_0000`

Address decoding is already done in `bus_interconnect.v`. `sel`
is asserted only for this peripheral. Look at `addr[7:2]` for
the register index — 64 registers available.

### Rule 3 — Interrupts are edge-latched

The interrupt controller latches a **rising edge** on each
source line into a pending bit. A one-cycle pulse is enough.

Level-held is also fine, **but the source register must clear
on software write.** If the line stays high after the handler
clears the INTC pending bit, the INTC sees a fresh edge and
re-fires immediately — the handler will loop forever.

Every peripheral in the subsystem uses this pattern:

```verilog
// hardware set wins over software clear
if (<condition>)
    flag <= 1'b1;
else if (wr && (reg_index == 6'hXX) && wdata[bit])
    flag <= 1'b0;

assign irq_rx = flag & irq_enable;
```

---

## 3. Interrupt IDs

| ID | Source | ID | Source |
|---|---|---|---|
| 0 | **CAN RX** | 5 | I2C |
| 1 | **CAN TX** | 6 | TIMER |
| 2 | **CAN ERROR** | 7 | ADC |
| 3 | UART | 8 | GPIO |
| 4 | SPI | | |

Lower ID wins when several are pending simultaneously, so CAN
has the highest priority of any source in the system.

> **Confirm this against Section 1.4.** These IDs were inferred
> from the statement that CAN occupies inputs 0-2 and GPIO is 8.
> If the real table differs, tell Person 2 — the only change is
> the concatenation order of `irq_sources` in
> `periph_subsystem.v`.

Software services an interrupt like this:

1. Trap to `mtvec`, read `mcause` — `0x8000000B` means external
2. Read `INTC.CAUSE` at `0x4008_0008` to get the IRQ ID
3. Service the peripheral and clear its own flag
4. Write 1 to the matching bit of `INTC.PENDING` at `0x4008_0004`
5. `mret`

---

## 4. System clock: 80 MHz

**Not 100 MHz.** This affects CAN bit timing directly.

Post-route timing measured the CPU's real fmax at ~93 MHz. The
limiting path is the single-cycle ALU plus three-way forwarding
in the execute stage. 80 MHz was chosen to leave margin.

Bit rate prescaler, assuming 16 time quanta per bit:

| Bit rate | Divider value |
|---|---|
| 125 kbit/s | 80e6 / (125e3 x 16) = 40, so 39 |
| 250 kbit/s | 80e6 / (250e3 x 16) = 20, so 19 |
| 500 kbit/s | 80e6 / (500e3 x 16) = 10, so 9 |
| 1 Mbit/s | 80e6 / (1e6 x 16) = 5, so 4 |

Make the prescaler a software-writable register rather than a
constant. Every other peripheral does, which is why the 100 to
80 MHz change cost nothing elsewhere.

---

## 5. Timing budget

The design currently closes with **+0.162 ns** of worst negative
slack at 80 MHz. That is positive but thin.

Adding the CAN controller may push it negative on the first run.
Usually this is the placer coasting rather than a real problem —
Vivado stops optimising once constraints are met, so a design
with spare margin gets placed loosely. Re-run implementation
with strategy `Performance_ExplorePostRoutePhysOpt` before
concluding anything is wrong.

What genuinely does cause trouble: deep combinational chains.
Specifically avoid a combinational CRC-15 across a whole frame,
and avoid combinational division or modulo anywhere. Both will
build long ripple chains. A single-cycle 32-bit divider in the
CPU cost -80 ns before it was made sequential.

---

## 6. Register map format

Match the format in `REGISTER_MAP.md` so the CAN section drops
straight in:

```
## CAN - 0x4007_0000

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | MODE | RW | [0] reset, [1] listen-only, ... |
| 0x04 | STATUS | RO | [0] tx_busy, [1] rx_valid, ... |
```

Note in that section which register write clears each interrupt
source — the trap handler needs it.

---

## 7. Testing before integration

Person 2 can run the CAN controller against
`tb_periph_subsystem.v` before it touches the CPU. That
testbench drives the bus master port directly, so it isolates
the peripheral completely.

This catches a combinational `prdata` immediately, rather than
after the failure is tangled up with pipeline behaviour. Worth
doing — send the module over whenever it elaborates cleanly,
even if the protocol logic is incomplete.

Add a section to that testbench following the existing pattern:

```verilog
localparam CAN = 32'h4007_0000;

bus_write(CAN + 32'h00, 32'h0000_0001);
bus_read (CAN + 32'h00);
check("can MODE readback", rdval, 32'h0000_0001);
```

---

## 8. Integration checklist

When the module is ready, Person 2 does the following in
`soc_top.v` — nothing inside `periph_subsystem.v` changes:

- [ ] Instantiate `can_controller`
- [ ] Connect the `can_*` port group to it
- [ ] Replace the `can_rdata` / `can_irq_*` tie-offs
- [ ] Add `can_rx` / `can_tx` to the top-level ports and XDC
- [ ] Run `tb_periph_subsystem` — expect ALL CHECKS PASSED
- [ ] Run `tb_top_level_riscv` — confirm no regression
- [ ] Re-run implementation, confirm timing still closes
