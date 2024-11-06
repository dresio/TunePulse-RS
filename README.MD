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
For more information, join the creapunk community [Discord](https://discord.gg/V4aJdTja8v).**

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