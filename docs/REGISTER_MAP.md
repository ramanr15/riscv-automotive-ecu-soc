# Peripheral Subsystem Register Map — Person 2

All peripherals sit at base `0x4000_0000`. Address bits `[19:16]` select the
peripheral, bits `[7:2]` select the register within it. All registers are
32-bit, word aligned. Use `lw` / `sw` only.

| Index | Peripheral | Base address |
|---|---|---|
| 0 | GPIO  | `0x4000_0000` |
| 1 | TIMER | `0x4001_0000` |
| 2 | PWM   | `0x4002_0000` |
| 3 | UART  | `0x4003_0000` |
| 4 | SPI   | `0x4004_0000` |
| 5 | I2C   | `0x4005_0000` |
| 6 | ADC   | `0x4006_0000` |
| 7 | CAN   | `0x4007_0000` (Person 1) |
| 8 | INTC  | `0x4008_0000` |

Data memory occupies `0x0000_0000 – 0x0FFF_FFFF` (4 KB, aliased).
Any access outside a defined region asserts `bus_error` rather than
silently returning zero.

## Interrupt IDs

| ID | Source | ID | Source |
|---|---|---|---|
| 0 | CAN RX | 5 | I2C |
| 1 | CAN TX | 6 | TIMER |
| 2 | CAN ERROR | 7 | ADC |
| 3 | UART | 8 | GPIO |
| 4 | SPI | | |

Lower ID wins when several are pending.

---

## INTC — `0x4008_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | MASK | RW | One enable bit per source |
| 0x04 | PENDING | RW | Latched requests, write 1 to clear |
| 0x08 | CAUSE | RO | Lowest pending & enabled ID (31 = none) |
| 0x0C | STATUS | RO | bit0 = any active |
| 0x10 | RAW | RO | Live, unlatched source lines |

## GPIO — `0x4000_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | DIR | RW | 1 = output |
| 0x04 | OUT | RW | Output value |
| 0x08 | IN | RO | Synchronised pin value |
| 0x0C | RISE_EN | RW | Per-pin rising edge detect |
| 0x10 | FALL_EN | RW | Per-pin falling edge detect |
| 0x14 | IRQ_PEND | RW | Write 1 to clear |
| 0x18 | IRQ_EN | RW | Per-pin contribution to the IRQ line |

## TIMER — `0x4001_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | RW | [0] enable, [1] auto-reload, [2] irq_en, [3] clear |
| 0x04 | PRESCALE | RW | [15:0] divider − 1 |
| 0x08 | COMPARE | RW | Period / compare value |
| 0x0C | COUNT | RO | Live counter |
| 0x10 | STATUS | RW | [0] match flag, write 1 to clear |

Interrupt period = `(PRESCALE+1) × (COMPARE+1)` clocks.

## PWM — `0x4002_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | RW | [0] enable, [1] invert |
| 0x04 | PRESCALE | RW | [15:0] divider − 1 |
| 0x08 | PERIOD | RW | Counter wrap value |
| 0x0C | COUNT | RO | Live counter |
| 0x10–0x1C | DUTY0–3 | RW | Per-channel duty |
| 0x20 | CHEN | RW | [3:0] per-channel output enable |

Output is high while `COUNT < DUTY`. Duty 0 = always low, duty > PERIOD = always high.

## UART — `0x4003_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | DATA | RW | Write pushes TX FIFO, read pops RX FIFO |
| 0x04 | STATUS | RO | [0] tx_full [1] tx_empty [2] rx_full [3] rx_empty [4] framing_err [5] overrun [6] tx_busy |
| 0x08 | OSDIV | RW | `f_clk / (16 × baud) − 1` |
| 0x0C | CTRL | RW | [0] rx irq_en, [1] tx irq_en, [2] clear errors |

8-N-1, 16-byte FIFOs each way. `OSDIV = 53` gives 115200 baud from 100 MHz.

## SPI — `0x4004_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | DATA | RW | Write starts a transfer, read returns the received byte |
| 0x04 | CTRL | RW | [7:0] clkdiv, [8] CPOL, [9] CPHA, [10] irq_en, [11] auto CS, [15:12] manual CS (active low) |
| 0x08 | STATUS | RW | [0] busy, [1] done (write 1 to clear) |

`SCLK = f_clk / (2 × (clkdiv + 1))`. Master mode, 8 bits, MSB first.

## I2C — `0x4005_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | RW | [15:0] clkdiv, [16] irq_en |
| 0x04 | CMD | WO | [0] START, [1] WRITE, [2] READ, [3] STOP, [4] ACK value for READ |
| 0x08 | TXDATA | RW | Byte to send (address byte = `{addr7, rw}`) |
| 0x0C | RXDATA | RO | Last received byte |
| 0x10 | STATUS | RW | [0] busy, [1] ack_error, [2] done (write 1 to clear) |

`SCL = f_clk / (4 × (clkdiv + 1))`. 100 MHz → 249 for 100 kHz, 61 for 400 kHz.
Commands within one CMD word execute in the order START → WRITE/READ → STOP,
so a whole "start, address, stop" sequence is one register write.
Clock stretching is handled automatically.

**Typical write to register `R` of slave `0x50`:**
1. `TXDATA = 0xA0` (0x50<<1 | 0), `CMD = START|WRITE` → poll DONE, check ack_error
2. `TXDATA = R`, `CMD = WRITE` → poll DONE
3. `TXDATA = data`, `CMD = WRITE|STOP` → poll DONE

## ADC — `0x4006_0000`

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | RW | [0] start (self-clearing), [1] irq_en, [7:4] channel |
| 0x04 | STATUS | RW | [0] busy, [1] done (write 1 to clear) |
| 0x08 | RESULT | RO | [15:0] last conversion |

## CSR additions (machine mode)

| CSR | Address | Notes |
|---|---|---|
| mie | 0x304 | Set bit 11 (MEIE) to allow external interrupts |
| mip | 0x344 | Bit 11 (MEIP) is read-only, mirrors the INTC output |

Enable interrupts with:
```
li   t0, (1<<11)
csrs mie, t0        # MEIE
li   t0, (1<<3)
csrs mstatus, t0    # MIE
```
The trap handler at `mtvec` (reset value `0x0000_0100`) reads `mcause`.
Bit 31 set with cause 11 means an external interrupt; read `INTC.CAUSE`
to find the source, service it, clear the peripheral flag, then write 1 to
`INTC.PENDING` for that bit, then `mret`.
