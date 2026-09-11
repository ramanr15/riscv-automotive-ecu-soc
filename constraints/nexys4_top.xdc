# ============================================================
# nexys4_top.xdc
#
# Nexys 4 / Nexys 4 DDR  (XC7A100T-1CSG324C)
# RISC-V Automotive ECU SoC
#
# Replaces timing_wrapper.xdc. Only one of the two may be
# active in the project at a time.
#
# CLOCK: the board oscillator is 100 MHz; clk_wiz_0 divides it
# to the 80 MHz the SoC actually runs at. The create_clock below
# constrains the BOARD pin, so it stays at 10 ns -- the MMCM
# output is constrained automatically from the IP's own settings.
# Do not add a second create_clock for the internal clock.
# ============================================================

# ------------------------------------------------------------
# Clock : 100 MHz oscillator
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports CLK100MHZ]
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} [get_ports CLK100MHZ]

# ------------------------------------------------------------
# Reset : centre pushbutton
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports BTNC]

# ------------------------------------------------------------
# GPIO inputs : slide switches SW0-SW15
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {SW[0]}]
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {SW[1]}]
set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {SW[2]}]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {SW[3]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {SW[4]}]
set_property -dict {PACKAGE_PIN T18 IOSTANDARD LVCMOS33} [get_ports {SW[5]}]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports {SW[6]}]
set_property -dict {PACKAGE_PIN R13 IOSTANDARD LVCMOS33} [get_ports {SW[7]}]
set_property -dict {PACKAGE_PIN T8  IOSTANDARD LVCMOS18} [get_ports {SW[8]}]
set_property -dict {PACKAGE_PIN U8  IOSTANDARD LVCMOS18} [get_ports {SW[9]}]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports {SW[10]}]
set_property -dict {PACKAGE_PIN T13 IOSTANDARD LVCMOS33} [get_ports {SW[11]}]
set_property -dict {PACKAGE_PIN H6  IOSTANDARD LVCMOS33} [get_ports {SW[12]}]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports {SW[13]}]
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports {SW[14]}]
set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports {SW[15]}]

# ------------------------------------------------------------
# GPIO outputs : LED0-LED15
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {LED[0]}]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {LED[1]}]
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {LED[2]}]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {LED[3]}]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {LED[4]}]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVCMOS33} [get_ports {LED[5]}]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports {LED[6]}]
set_property -dict {PACKAGE_PIN U16 IOSTANDARD LVCMOS33} [get_ports {LED[7]}]
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports {LED[8]}]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {LED[9]}]
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {LED[10]}]
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports {LED[11]}]
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {LED[12]}]
set_property -dict {PACKAGE_PIN V14 IOSTANDARD LVCMOS33} [get_ports {LED[13]}]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {LED[14]}]
set_property -dict {PACKAGE_PIN V11 IOSTANDARD LVCMOS33} [get_ports {LED[15]}]

# ------------------------------------------------------------
# Status LEDs : the RGB LED cathodes, used as plain LEDs
#   [0] MMCM locked
#   [1] SoC out of reset
#   [2] interrupt taken (stretched)
#   [3] BUS ERROR latched - software hit an unmapped address
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN R12 IOSTANDARD LVCMOS33} [get_ports {LED16_19[0]}]
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports {LED16_19[1]}]
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports {LED16_19[2]}]
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33} [get_ports {LED16_19[3]}]

# ------------------------------------------------------------
# UART : USB-serial bridge
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports UART_TXD_IN]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports UART_RXD_OUT]

# ------------------------------------------------------------
# Pmod JA : SPI
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports JA1_SPI_SCLK]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports JA2_SPI_MOSI]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports JA3_SPI_MISO]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports {JA_SPI_CS_N[0]}]
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports {JA_SPI_CS_N[1]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {JA_SPI_CS_N[2]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {JA_SPI_CS_N[3]}]

# ------------------------------------------------------------
# Pmod JB : PWM
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS33} [get_ports {JB_PWM[0]}]
set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33} [get_ports {JB_PWM[1]}]
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS33} [get_ports {JB_PWM[2]}]
set_property -dict {PACKAGE_PIN H14 IOSTANDARD LVCMOS33} [get_ports {JB_PWM[3]}]

# ------------------------------------------------------------
# Pmod JC : I2C
# EXTERNAL PULL-UPS REQUIRED (4.7k to 3V3). The internal PULLUP
# below is weak (tens of k) and is a convenience for bench
# testing only -- it will not meet I2C rise-time requirements.
# ------------------------------------------------------------
set_property -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports JC1_I2C_SCL]
set_property -dict {PACKAGE_PIN F6 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports JC2_I2C_SDA]

# ------------------------------------------------------------
# Pmod JD : CAN transceiver (e.g. MCP2551, SN65HVD230)
# ------------------------------------------------------------
# PULLUP so a disconnected pin reads recessive (CAN idle = 1).
# Without it, clearing the loopback bit with no transceiver fitted
# leaves the input floating and the controller sees random
# dominant bits, generating continuous error frames.
set_property -dict {PACKAGE_PIN H4 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports JD1_CAN_RX]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33} [get_ports JD2_CAN_TX]

# ============================================================
# Timing exceptions
#
# Every input below is double-flopped inside the design, and no
# external interface has an agreed timing budget yet. These
# exceptions keep the report focused on internal
# register-to-register paths, which is what actually limits fmax.
#
# If a real SPI or I2C device is later attached and its setup or
# hold times matter, replace the relevant line with a proper
# set_input_delay / set_output_delay.
# ============================================================
set_false_path -from [get_ports BTNC]
set_false_path -from [get_ports {SW[*]}]
set_false_path -from [get_ports UART_TXD_IN]
set_false_path -from [get_ports JA3_SPI_MISO]
set_false_path -from [get_ports JC1_I2C_SCL]
set_false_path -from [get_ports JC2_I2C_SDA]
set_false_path -from [get_ports JD1_CAN_RX]

set_false_path -to [get_ports {LED[*]}]
set_false_path -to [get_ports {LED16_19[*]}]
set_false_path -to [get_ports UART_RXD_OUT]
set_false_path -to [get_ports JA1_SPI_SCLK]
set_false_path -to [get_ports JA2_SPI_MOSI]
set_false_path -to [get_ports {JA_SPI_CS_N[*]}]
set_false_path -to [get_ports {JB_PWM[*]}]
set_false_path -to [get_ports JC1_I2C_SCL]
set_false_path -to [get_ports JC2_I2C_SDA]
set_false_path -to [get_ports JD2_CAN_TX]

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
