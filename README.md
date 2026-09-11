# RISC-V Automotive ECU SoC

An RV32IM system-on-chip for an automotive electronic control unit,
implemented in Verilog-2001 and running on a Digilent Nexys 4 DDR
(Xilinx XC7A100T-1CSG324C) at 80 MHz.

The CPU is surrounded by a custom bus interconnect, a priority
interrupt controller, seven memory-mapped peripherals and a CAN 2.0B
controller. Verified in simulation at three levels and confirmed on
hardware.

---

## Status

| | |
|---|---|
| Clock | 80 MHz (post-route fmax ≈ 93 MHz) |
| Timing | WNS **+0.233 ns**, 0 failing endpoints of 10 919 |
| Utilisation | 3896 LUT (6%), 3425 FF, 4 DSP, 1 BRAM, 55 I/O |
| Power | 0.149 W estimated |
| Hardware | CAN loopback frame received and displayed on LEDs |

---

## Architecture

```
                    nexys4_top
                        |
        +---------------+----------------+
        |                                |
   clk_wiz_0                          soc_top
   100 -> 80 MHz          +--------------+--------------+
                          |              |              |
                   top_level_riscv  can_adapter   periph_subsystem
                     (RV32IM)            |               |
                                  can_controller_top   bus_interconnect
                                                       intc
                                                       GPIO / Timer / PWM
                                                       UART / SPI / I2C / ADC
                                                       data_mem
```

### Memory map

Peripherals occupy `0x4000_0000`, selected by address bits `[19:16]`.
Data memory sits at `0x0000_0000`. Anything else raises a bus error
rather than silently returning zero.

| Base | Peripheral | | Base | Peripheral |
|---|---|---|---|---|
| `0x4000_0000` | GPIO | | `0x4005_0000` | I2C |
| `0x4001_0000` | Timer | | `0x4006_0000` | ADC |
| `0x4002_0000` | PWM | | `0x4007_0000` | CAN |
| `0x4003_0000` | UART | | `0x4008_0000` | Interrupt controller |
| `0x4004_0000` | SPI | | | |

Full register tables in [`docs/REGISTER_MAP.md`](docs/REGISTER_MAP.md).

### Interrupts

Nine sources, fixed priority, lowest ID wins. Edge-latched so a
single-cycle pulse is never lost, with a write-1-to-clear pending
register.

| ID | Source | | ID | Source |
|---|---|---|---|---|
| 0 | CAN RX | | 5 | I2C |
| 1 | CAN TX | | 6 | Timer |
| 2 | CAN error | | 7 | ADC |
| 3 | UART | | 8 | GPIO |
| 4 | SPI | | | |

The controller drives a single line into the CPU's external interrupt
input via `mie.MEIE` / `mip.MEIP`.

---

## Bus contract

One rule governs every peripheral:

> **Request on cycle N, registered read data on cycle N+1.**

There is no `ready` signal and the CPU has no memory-stall path, so a
combinational read returns corrupt data. Slow protocols (SPI, I2C, CAN)
run asynchronously behind busy/done status bits and interrupts; the
register access itself always completes in one cycle.

This is documented for integrators in
[`docs/CAN_INTEGRATION_SPEC.md`](docs/CAN_INTEGRATION_SPEC.md).

---

## Repository layout

```
rtl/          synthesisable Verilog-2001
  cpu/          RV32IM core
  peripherals/  bus, interrupt controller, 7 peripherals, FIFO template
  can/          CAN 2.0B controller
  soc_top.v     CPU + CAN + peripherals
  nexys4_top.v  board wrapper
sim/          testbenches
constraints/  Nexys 4 DDR pin and timing constraints
sw/           test programs and the assembler that generates them
docs/         register map, integration spec, bring-up guide
```

---

## Building

Vivado 2025.2, part `xc7a100t-csg324-1`.

1. Create a project by **part number** (not by board) and add
   everything under `rtl/`.
2. IP Catalog → **Clocking Wizard**, component name `clk_wiz_0`:
   100 MHz in, **80.000 MHz** `clk_out1`, `locked` and active-high
   `reset` enabled.
3. Add `constraints/nexys4_top.xdc`, set `nexys4_top` as top.
4. Add a program from `sw/` as `program.mem`, in **Design Sources**
   (not only Simulation Sources — `$readmemh` runs during synthesis).
5. Synthesis → Implementation → Generate Bitstream.

`Def.v` is included by several files; add its directory under
Settings → Verilog options → Include Directories.

---

## Verification

| Level | Testbench | Covers |
|---|---|---|
| Peripherals | `sim/tb_periph_subsystem.v` | Bus decode, unmapped-access error, all 7 peripherals, interrupt controller, CAN register access and a full loopback frame |
| System | `sim/tb_top_level_riscv.v` | CPU with peripherals, timer interrupt, trap entry and `mret` |
| Hardware | `sw/program_can_test.mem` | The complete path on silicon |

The hardware test transmits one CAN frame in loopback, takes the RX
interrupt, and writes the received identifier to the LEDs. **`0x245` on
LED[15:0]** means bus decode, CAN framing, the interrupt controller and
the CSR trap path all work together.

Step-by-step procedure in
[`docs/FPGA_Bringup_Guide.pdf`](docs/FPGA_Bringup_Guide.pdf).

---

## Engineering notes

Three findings worth recording, since each shaped the design.

### Timing closure: −80 ns to +0.233 ns

Initial synthesis failed catastrophically. The critical path was 309
logic levels, 293 of them CARRY4 — a 32-bit divider implemented
combinationally in a single cycle.

| Change | WNS |
|---|---|
| Initial synthesis @ 100 MHz | −80.152 ns |
| Sequential divider + split multiplier | −1.499 ns |
| Operand capture stage before the DSPs | −1.011 ns |
| Post-route | −0.747 ns |
| Retargeted to 80 MHz, post-route | **+0.233 ns** |

`muldiv_unit` now uses restoring division (35 cycles) and a partial-
product multiplier (4 cycles), with an EX-stall handshake added to the
pipeline. The remaining path — register file → forwarding mux → ALU →
result mux — is inherent to a single-cycle execute stage on a −1 speed
grade part. 80 MHz was chosen with margin rather than restructuring the
CPU.

Slack varies by a few hundred picoseconds between implementation runs.
At 5% device utilisation the placer stops optimising as soon as
constraints are met, so the default strategy occasionally lands
slightly negative; `Performance_ExplorePostRoutePhysOpt` closes it
reliably. The design has roughly 1.5 ns of genuine margin at 80 MHz —
the reported figure is placement noise, not headroom.

Timing is independent of which program is loaded: `program.mem` only
initialises the instruction memory array and does not change the
logic.

### CAN read-timing mismatch

`can_regfile.v` drives read data combinationally, one cycle earlier
than the bus contract requires. Neither team's testbench could detect
it: the CAN testbench samples `rdata` in the same cycle it is asserted,
so it *requires* the combinational behaviour.

`can_adapter.v` bridges this at the SoC boundary, registering the read
data and preserving the pop-on-read side effect on `CAN_RX_CTRL`.
Verified through the real interconnect with back-to-back reads.

The proper fix is a clocked read mux in `can_regfile.v`, at which point
`REGISTER_RDATA` can be set to 0 and the shim becomes a pass-through.

### Known CPU bug: distance-2 load-use forwarding

`top_level_riscv.v` forwards `execute_result_delay` for a register
produced two instructions earlier. For a load that register holds the
memory **address**, not the loaded data — `mem_to_reg_delay` exists
alongside it and is never consulted. `hazard_unit` only stalls the
distance-1 case.

Programs in `sw/` insert three delay slots after any load whose result
is used. The one-line RTL fix is:

```verilog
else if (forward_delay_a)
    forwarded_a = mem_to_reg_delay ? formatted_load_data
                                   : execute_result_delay;
```

Not yet applied, to avoid re-verifying a timing-closed design.

---

## Scope limits

Documented rather than silently missing:

- **ADC** — the wrapper exists; the XADC needs a DRP wrapper that does not.
- **CAN** — `hard_sync`/`resync_edge` and `bit_error` are tied off. Fine
  for single-clock-domain and loopback testing, not for full conformance.
- **I2C** — needs external 4.7 kΩ pull-ups. The internal ones in the XDC
  are a bench convenience and will not meet rise-time requirements.
- **Instruction memory** — distributed RAM, not BRAM. The asynchronous
  read prevents BRAM inference; a registered fetch would add an IF stage.

---

## Team

| | |
|---|---|
| CAN controller RTL, embedded software stack | Rama Krishna Prasadh H |
| Bus, interrupt controller, peripherals, SoC integration, FPGA bring-up | Raman R |
| Verification | B Kavin |

Department of ECE, BMS Institute of Technology and Management
(Autonomous under VTU), Bengaluru.
