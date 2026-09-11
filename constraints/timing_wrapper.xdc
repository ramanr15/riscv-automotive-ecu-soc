# ============================================================
# timing_wrapper.xdc
#
# Pin assignments for Nexys 4 / Nexys 4 DDR (XC7A100T-1CSG324C)
# so Implementation can run and report real post-route timing.
#
# GPIO goes to Pmod JA/JB because those are genuinely
# bidirectional. The slide switches are input-only in practice
# and driving them from an inout port risks contention.
#
# Replace this whole file with the real board constraints once
# nexys4_top exists.
#
# ------------------------------------------------------------
# CLOCK CHOICE : 80 MHz
#
# The board oscillator is 100 MHz, but the CPU cannot run at
# that rate. Post-route timing measured a worst negative slack
# of -0.747 ns against a 10.000 ns period, so the longest path
# takes 10.75 ns and real fmax is about 93 MHz.
#
# The limiting path is register file -> forwarding mux -> ALU
# -> result mux -> pipeline register. That is inherent to a
# single-cycle execute stage with three-way forwarding on a -1
# speed grade part; beating it would mean splitting EX across
# two pipeline stages, which is a CPU redesign.
#
# 12.500 ns (80 MHz) leaves roughly 1.75 ns of margin, enough
# to absorb the CAN controller and board-level routing without
# revisiting this.
#
# clk_wiz_0 must be configured to output 80 MHz to match.
#
# Peripheral dividers change with the clock. Reset defaults in
# the RTL should be updated to suit:
#     uart_ctrl.v   osdiv  <= 16'd42    (115200 baud)
#     i2c_ctrl.v    clkdiv <= 16'd199   (100 kHz)
#                   clkdiv <= 16'd49    (400 kHz)
# ============================================================

# ---------------- Clock : 80 MHz system clock ----------------
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 12.500 -name sys_clk -waveform {0.000 6.250} [get_ports clk]

# ---------------- Reset : centre pushbutton ----------------
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports reset]

# ---------------- UART : USB-serial bridge ----------------
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports uart_tx]

# ---------------- GPIO[7:0] : Pmod JA ----------------
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[0]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[1]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[2]}]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[3]}]
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[4]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[5]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[6]}]
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[7]}]

# ---------------- GPIO[15:8] : Pmod JB ----------------
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[8]}]
set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[9]}]
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[10]}]
set_property -dict {PACKAGE_PIN H14 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[11]}]
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[12]}]
set_property -dict {PACKAGE_PIN F13 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[13]}]
set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[14]}]
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[15]}]

# ---------------- SPI : Pmod JC ----------------
set_property -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33} [get_ports spi_sclk]
set_property -dict {PACKAGE_PIN F6 IOSTANDARD LVCMOS33} [get_ports spi_mosi]
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports spi_miso]
set_property -dict {PACKAGE_PIN G6 IOSTANDARD LVCMOS33} [get_ports {spi_cs_n[0]}]
set_property -dict {PACKAGE_PIN E7 IOSTANDARD LVCMOS33} [get_ports {spi_cs_n[1]}]
set_property -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports {spi_cs_n[2]}]
set_property -dict {PACKAGE_PIN J4 IOSTANDARD LVCMOS33} [get_ports {spi_cs_n[3]}]

# ---------------- I2C : Pmod JD ----------------
# Open drain in RTL; add external pull-ups on the real board.
set_property -dict {PACKAGE_PIN H4 IOSTANDARD LVCMOS33} [get_ports i2c_scl]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33} [get_ports i2c_sda]

# ---------------- PWM : LED[3:0] ----------------
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {pwm_out[0]}]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {pwm_out[1]}]
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {pwm_out[2]}]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {pwm_out[3]}]

# ---------------- CAN transceiver : Pmod JD ----------------
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports can_rx]
set_property -dict {PACKAGE_PIN G3 IOSTANDARD LVCMOS33} [get_ports can_tx]

# ---------------- Reduced debug : LED[4] ----------------
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports debug_xor]

# ---------------- Asynchronous inputs ----------------
# All double-flopped inside the design.
set_false_path -from [get_ports reset]
set_false_path -from [get_ports uart_rx]
set_false_path -from [get_ports spi_miso]
set_false_path -from [get_ports i2c_scl]
set_false_path -from [get_ports i2c_sda]
set_false_path -from [get_ports {gpio_pins[*]}]
set_false_path -from [get_ports can_rx]

# ---------------- Outputs ----------------
# No external interface budget yet; relax so the report stays
# focused on internal register-to-register paths.
set_false_path -to [get_ports uart_tx]
set_false_path -to [get_ports spi_sclk]
set_false_path -to [get_ports spi_mosi]
set_false_path -to [get_ports {spi_cs_n[*]}]
set_false_path -to [get_ports {pwm_out[*]}]
set_false_path -to [get_ports {gpio_pins[*]}]
set_false_path -to [get_ports can_tx]
set_false_path -to [get_ports debug_xor]
