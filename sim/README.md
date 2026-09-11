# Testbenches

## SoC level

| File | Simulation top | Covers |
|---|---|---|
| `tb_periph_subsystem.v` | yes | Bus decode, unmapped-access error, data memory, GPIO edge interrupts, timer, PWM, UART loopback, SPI loopback, interrupt controller priority, **CAN register access through the real bus**, and a full CAN loopback frame |
| `tb_top_level_riscv.v` | yes | CPU with the peripheral subsystem attached: timer interrupt, trap entry, `mret`, and a pipeline execution table |

`tb_periph_subsystem` is the more thorough of the two and should be run
first. It drives the bus master port directly, so a failure points at
one peripheral rather than at the CPU.

Sections 9 and 10 are the CAN integration checks. Section 9's
back-to-back `CAN_IE` write-read-write-read is the one that matters: it
catches a read-timing shim that is off by one cycle, which a single
read would pass by luck.

## CAN module level

`tb_can_*.v` are Person 1's per-module benches, kept for reference.
Each has its own top and tests one module in isolation.

Note that `tb_can_controller_top.v` samples `rdata` in the same cycle
it asserts `re`, so it requires the combinational read in
`can_regfile.v` and structurally cannot detect the mismatch with the
SoC bus contract. That is what `can_adapter.v` exists to bridge, and
what section 9 of `tb_periph_subsystem` verifies.
