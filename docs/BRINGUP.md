# FPGA Bring-Up Checklist

Nexys 4 / Nexys 4 DDR (XC7A100T-1CSG324C), 80 MHz.

## 1. Reconfigure clk_wiz_0 — do this first

The old wrapper used 75 MHz. The SoC needs **80 MHz**.

IP Catalog → Clocking Wizard → re-customise `clk_wiz_0`:

- Input: 100.000 MHz, single-ended
- `clk_out1`: **80.000** MHz
- `locked` output: enabled
- `reset` input: enabled, active high

Everything else defaults. Regenerate the IP.

If this is skipped, the design still builds but every peripheral
divider is wrong: UART baud, I2C rate and CAN bit rate all scale
with the clock.

## 2. Swap the top module

**Design Sources**
- Add `nexys4_top.v`, set as **Top**
- Keep everything else; `soc_top` is now instantiated by it
- Remove `timing_wrapper.v` — it was only ever a scaffold to get
  a real timing number past the 260-pin problem

**Constraints**
- Remove `timing_wrapper.xdc`
- Add `nexys4_top.xdc`

Only one of the two constraint files may be active. Both define a
clock on the same pin.

## 3. Program memory

Replace `program.mem` with `program_can_test.mem` (rename it to
`program.mem` — `instr_mem` reads that filename literally).

Keep the old one. It is still the CPU regression test.

Make sure it is in **Design Sources**, not only Simulation
Sources. `$readmemh` runs during synthesis to initialise the
memory; if Vivado cannot find the file the bitstream contains an
instruction memory full of NOPs and the board does nothing.

## 4. Build

Run Synthesis → Implementation → Generate Bitstream.

Expected: around 3900 LUTs (6%), 4 DSP, 1 BRAM, 39 pins, WNS
positive at 12.5 ns. If WNS comes back slightly negative, re-run
with strategy `Performance_ExplorePostRoutePhysOpt` before
changing anything — the placer coasts once constraints are met.

## 5. What the board should do

The test program enables the CAN controller in **loopback**, so no
transceiver is needed on Pmod JD for this test. It transmits one
frame (ID `0x245`, data `DEADBEEF 12345678`, DLC 8), which comes
straight back, raises CAN RX (IRQ 0), and the trap handler writes
the received ID to the GPIO output register.

After programming:

| Indicator | Expected |
|---|---|
| LED16 | on — MMCM locked |
| LED17 | on — SoC out of reset |
| LED18 | one brief flash — the CAN RX interrupt |
| LED19 | **off** — a lit LED19 means an unmapped bus access |
| LED[15:0] | `0000 0010 0100 0101` = 0x245, the received CAN ID |

`0x245` on the LEDs is the whole test: it means the CPU
configured the CAN controller over the bus, a real frame was
serialised onto the wire and decoded back, the interrupt reached
the CPU through the controller, and the handler read the RX
registers correctly.

Press BTNC to reset and it repeats.

## 6. If something is wrong

**All LEDs dark, LED16 off** — clk_wiz_0 is not locking. Check it
was regenerated for 80 MHz and that `CLK100MHZ` is on E3.

**LED16 on, everything else dark** — the CPU is not running, or
`program.mem` was not found at synthesis. Check the synthesis log
for a `$readmemh` warning, and confirm the file is in Design
Sources.

**LED19 lit** — software touched an unmapped address. With this
program that should be impossible, so it points at a corrupt
`program.mem` or a memory that failed to initialise.

**LEDs show something other than 0x245** — the interrupt fired but
the RX data is wrong. Attach an ILA to `debug_pc` and
`debug_wb_data` (both already carry `mark_debug`) and check
whether the handler is reading the right addresses.

**LED18 never flashes** — no interrupt. The frame is not
completing. Most likely the CAN bit timing does not match the
80 MHz clock, which again points back at step 1.

## 7. Once loopback works

Clear `CAN_CTRL` bit 1 to leave loopback, attach a real
transceiver (MCP2551 or SN65HVD230) to JD1/JD2, and the same
program transmits onto a real bus. At that point the bit rate
matters:

| Bit rate | BRP (BTIME[7:0]) at 80 MHz, 7 Tq/bit |
|---|---|
| 125 kbit/s | 90 |
| 250 kbit/s | 44 |
| 500 kbit/s | 22 |
| 1 Mbit/s | 10 |

The test program uses BRP = 0, which is only valid for loopback.

## 8. I2C needs external pull-ups

The XDC enables the FPGA's internal weak pull-ups as a bench
convenience. They are tens of kilohms and will not meet I2C
rise-time requirements. Fit 4.7k resistors to 3V3 on JC1/JC2
before trusting any I2C result.
