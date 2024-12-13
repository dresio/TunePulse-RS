# TunePulse Firmware based on Rust

[![License](https://img.shields.io/badge/LICENSE-Apache_2.0-blue.svg)](https://github.com/creapunk/TunePulse/blob/main/LICENSE)
 [![Rust](https://img.shields.io/badge/Rust-white?logo=rust&logoColor=%23F5822A)](https://www.rust-lang.org/)
 [![Discord](https://img.shields.io/discord/1098363068435681290?style=social&logo=discord&label=COMMUNITY)](https://discord.gg/V4aJdTja8v)
 [![Ko-fi](https://img.shields.io/badge/Support%20on%20Ko--fi-F16061?style=flat&logo=kofi&logoColor=white&labelColor=%23FF5E5B)](https://ko-fi.com/creapunk)

**TunePulse** is an upcoming open-source firmware designed to:

- Control a wide range of motors (`DC`, `STEPPERS`, `BLDC`, `PMSM`, `LINEAR`)
- Operate across various control modes (`CLOSED-LOOP`, `FOC`, `POSITION`, `SPEED`, `TORQUE`, etc.)
- Feature compatibility with multiple interfaces (`STEP-DIR`, `UART`, `I2C`, `CAN/CAN-FD`, `USB`)
- Support various protocols (`GCODE`, `KLIPPER`, etc.)

Its primary goal is to advance the development of closed-loop systems and simplify their integration into existing 32bit MCU applications.

> **Note**: This firmware is under active development and currently lacks documentation.
>  For more information, join the Creapunk community on [Discord](https://discord.gg/V4aJdTja8v).

------

## Getting Started

To get started with **TunePulse**, ensure you have the required Rust tools installed:

```bash
rustup target add thumbv7em-none-eabihf
cargo install flip-link
```

For additional setup instructions, see the [Probe.rs Installation Guide](https://probe.rs/docs/getting-started/installation/) and [Probe Setup Guide](https://probe.rs/docs/getting-started/probe-setup/#st-link).

1. Connect the ST-Link port to your computer and run the updater downloaded from the official ST-Link website.
2. Plug the controller into the programmer's single port and flash the blink example for testing:

```bash
cargo flash --release --package blink --chip STM32G431CBTx
```

After flashing, the controller should cycle through the Red, Green, and Blue LEDs.

**Optional**: Install the [Probe.rs Debugger extension for VSCode](https://marketplace.visualstudio.com/items?itemName=probe-rs.probe-rs-debugger).

------

## Flashing

1. Connect your computer to the ST-Link port on the programmer.
2. Connect the controller to the programmer's single port.
3. Run the following command to flash main firmware to CLN17 V2:

```bash
cargo run --package app
```

or this if you whant to get access to debug messages (tested on Windows PowerShell)

```bash
$env:DEFMT_LOG = "debug"; cargo run --package app
```

------

## Progress

### Algorithms

- ☑️ PID controller with a FeedForward component (integer and floating-point implementations)
- ☑️ Integer-math LPF filter with a variant for handling zero-crossing transitions
- ☑️ Encoder coordinate system with zero-crossing handling and full rotation counting
- ☑️ Calculation of motor instantaneous speed using a circular buffer
- ☑️ Normalization of ADC input channels relative to `vref`
- ☑️ Calculation of supply voltage with filtering
- ☑️ Motor type selection algorithm via selector:
	- ☑️ DC motor
	- ☑️ Stepper motor
	- ☑️ BLDC motor (SVPWM with limiting for insufficient supply voltage)
- ☑️ Phase commutation algorithm based on a predefined pattern
- ☑️ Fast sine/cosine calculation via lookup table

### Calibration
- ☑️ Pole count calculation
- ☑️ Lookup calibration table

### Drivers

- ☑️ PWM center-aligned timer
- ☑️ Encoder readings with DMA
- ☑️ ADC voltage and current readings with DMA