# TunePulse Firmware based on Rust

[![License](https://img.shields.io/badge/LICENSE-Apache_2.0-blue.svg)](https://github.com/creapunk/TunePulse/blob/main/LICENSE)
[![Rust](https://img.shields.io/badge/Rust-white?logo=rust&logoColor=%23F5822A)](https://www.rust-lang.org/)
[![Discord](https://img.shields.io/discord/1098363068435681290?style=social&logo=discord&label=COMMUNITY)](https://discord.gg/V4aJdTja8v)
[![Ko-fi](https://img.shields.io/badge/Support%20on%20Ko--fi-F16061?style=flat&logo=kofi&logoColor=white&labelColor=%23FF5E5B)](https://ko-fi.com/creapunk)

**TunePulse** is an upcoming open-source firmware designed to

- control a wide range of motors (`DC`, `STEPPERS`, `BLDC`, `PMSM`, `LINEAR`)
- across various control modes (`CLOSED-LOOP`, `FOC`, `POSITION`, `SPEED`, `TORQUE`, etc.)
- featuring compatibility with multiple interfaces (`STEP-DIR`, `UART`, `I2C`, `CAN/CAN-FD`, `USB`)
- and support for various protocols (`GCODE`, `KLIPPER`, etc.).

Its primary goal is to advance the development of closed-loop systems and simplify their integration into existing systems.

> **Note: This firmware is under heavy development and currently has no documentation.
> For more information, join the creapunk community [Discord](https://discord.gg/V4aJdTja8v).**

## Getting Started

```
rustup target add thumbv7em-none-eabihf
cargo install flip-link
```

https://probe.rs/docs/getting-started/installation/
https://probe.rs/docs/getting-started/probe-setup/#st-link

Plug in just to ST-Link port and run the downloaded updater from the ST-Link website  
Plug in the controller to the single port on the programmer and flash the blink example for testing:

```
cargo flash --release --package blink --chip STM32G431CBTx
```

The controller should now be cycling from the Red, Green, and Blue LEDs.

Optional: Install the probe-rs debugger extension for VSCode  
https://marketplace.visualstudio.com/items?itemName=probe-rs.probe-rs-debugger

## Flashing

Plug in The computer to ST-LINK Port on the programmer, and the controller to the single port on the programmer.

cargo flash --release --chip STM32G431CBTx

## Progress

- ☑️ PID controller with a FeedForward component (integer and float implementations)
- ☑️ Integer math LPF filter + variation for handling transitions through zero
- ☑️ Encoder coordinate system with zero-crossing handling and full rotation counting
- ☑️ Calculation of motor instantaneous speed using a circular buffer
- ☑️ Normalization of ADC input channels relative to vref
- ☑️ Calculation of supply voltage with filtering
- ☑️ Motor type selection algorithm via selector
  - ☑️ DC motor
  - ☑️ Stepper motor
  - ☑️ BLDC motor (SVPWM, limiting when supply voltage is insufficient)
- ☑️ Phase commutation algorithm according to pattern
- ☑️ Fast sine/cosine via lookup table

- [ ] PWM Generation on tim2 (center aligned)
- [ ] DMA with ADC
- [ ] SPI for encoder (Half Duplex Master Mode, no miso, only mosi (may be good to make this configurable))
- [ ] Extra: Bring in probe.rs for debugging

cargo flash --release --package pwm_test --chip STM32G431CBTx
