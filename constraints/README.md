# Constraints

Only **one** of these may be active in the project at a time. Both
define a clock on the same pin.

## `nexys4_top.xdc` — the real one

Nexys 4 / Nexys 4 DDR pin assignments for `nexys4_top`. 55 pins:
switches, LEDs, four status LEDs, UART, and Pmods JA (SPI), JB (PWM),
JC (I2C) and JD (CAN).

The `create_clock` constrains the 100 MHz board oscillator on E3. The
80 MHz SoC clock is derived automatically from the `clk_wiz_0`
settings — do not add a second `create_clock` for it.

I2C has the internal weak pull-ups enabled as a bench convenience.
**External 4.7 kΩ pull-ups to 3V3 are required** for real I2C rise
times.

## `timing_wrapper.xdc` — scaffolding

Pairs with `rtl/timing_wrapper.v`. Kept because it is how the real
post-route timing number was obtained: `soc_top` exposes 260 top-level
pins against 210 available, so Place Design fails before it reaches
timing. The wrapper reduces this to 36 by driving the dangling inputs
from an LFSR and XOR-reducing the outputs to one pin — constants would
have let synthesis optimise away the very logic being measured.

Not needed for the board build.
