# Change Log

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](http://semver.org/).


## [v0.7.1] - 2022-04-11

### Fixed

    - Shorten old buffer when we have extra characters in DMA buffer after a character match.
## [v0.7.0] - 2022-04-04

### Added

    - Add delay implementation based on `cortex_m::asm::delay`.
    - Implement `RngCore` and `CryptoRng` for the hardware RNG.
    - Support analog to digital converters (ADC).
    - Support SPI slave mode.
    - Add DMA and interrupt support for SPI and ADC peripherals.
    - Add support for measuring Vref, Vbat and temperature.
    - Add alternate function 0 to GPIO.
    - Add preliminary bxCAN support.
    - Add more GPIO combinations for I2C and PWM peripherals.
    - Add wakeup clock sources to RTC config.
    - Support RTC domain backup registers.
    - Support I2C on stm32l4x3 devices.
    - Support I2C3 peripheral on stm32l4x6 devices.
    - Experimental support for the Synopsis USB library.
    - Experimental support for USB OTG FS for stm32l4x5 and stm32l4x6 devices.
    - Support stm32l4r9 devices.

### Changed

    - Use device-specific features rather than by-peripheral features.
    - Use `fugit` duration nd rate units instead of custom
    - Use const-generics for GPIO (require Rust 1.51)
    - Import I2C implementation from `stm32h7xx-hal` crate.
    - Use a `Config` struct for initializing I2C peripherals.
    - Check that the clock requested for the low-power timer is enabled.
    - Take `clocks` argument by value when setting up low-power timer.
    - Use sane low-power timer defaults (LSI, no prescaler).
    - Make `LowPowerTimer<_>::set_autoreload()` public.
    - Enable SPI2 for all stm32l4x2 devices as some of them have it.
    - Target hardfp by default since stm32l4 cores are Cortex-M4F.
    - Require typed input when converting from milliseconds to hertz.
    - Rework alternate function typestates.
    - Use MSI as default/fallback clock source for SYSCLK.
    - Use specialized PAC for stm32l412 and stm32l422 devices.
    - Add `toggeable` trait to GPIO pins.
    - Update `stm32l4` dependency.

### Fixed

    - Fix TIM1 PWM frequency computation.
    - Fix TIM5 counter width.
    - Fix PSC computation off-by-one error.
    - Change wait states values according to datasheet.
    - Fix incorrect I2C2 on PC0/PC1 on stm32l4x3 devices.
    - Swap QSPI pins and remove conflicting/wrong implementations.
    - Add power on GPIOG pins.
    - Support 0 byte writes on I2C.

## [v0.6.0] - 2020-12-11

### Added

    - USB driver for `stm32l4x2` and `stm32l4x3` devices.
    - USART3 and UART4, with dma support.
    - Support for GPIOF and GPIOG.
    - USB driver examples.
    - Support for external pin interrupts (EXTI).
    - Implement hardware flow control for serial.
    - Support for CRC peripheral.
    - Implement `core::hash::Hasher` for the CRC peripheral.
    - Support for serial framing.
    - Support for Half-Duplex Serial.
    - Support for timers 5 and 6.
    - Bump stm32l4 to `0.11.0`.
    - Support for QSPI.
    - Support for RTC alarms.
    - Support for the Independant Watch Dog Timer.
    - VSCode configurations.
    - Added signature module.


## Changed
    - Reworked clock control with RCC.
    - Rework the DMA API.
    - Rework the FLASH programming API.
    - Rework GPIO output to support setting Speed, Pullups and drain functionality.
    - Switched to Github actions.

### Fixed
    - Re-add support for GPIOD & GPIOE.
    - Fix enable bits for PWM.
    - Fix clearing serial error flag bits.
    - I2C 7bit adressing shift.
    - Fix the systick delay to support delays greater than `1 << 24`
    - Clean up examples, and Deps.

## [v0.5.0] - 2019-09-02

### Added

    - DMA2

### Breaking

    - Serial ports now use a config, instead of just the baudrate.
    - Upgraded embedded-hal, gpio method signitures are now fallible.
    - Bumped cortex-m.


## [v0.4.0] - 2019-05-08

### Added

    - PWM support for select timers
    - More GPIO support

### Fixed

    - More debug and Eq derives where appropriate
    - Updated to stm32-rs v0.7.0 which fixes [#32](https://github.com/stm32-rs/stm32l4xx-hal/issues/32) for stm32l4x5 and stm32l4x6 devices

## [v0.3.6] - 2019-02-11

### Added

    - GPIOE Support
    - MSI Clock support

### Fixed

    - The TSC `aquire` did not reset the channel select register after aquisition
    - The default charge high and charge low time for the TSC were fixed at the max, this now has a sane default and can be configured with the TSC config.
    - HCLK and PCKL rcc prescaling, although setting the right clocks, the value in `Clocks` was wrong, this now correctly calculated.


## [v0.3.5] - 2019-01-07

### Added

    - Rng implementation
    - GPIOE support
    - `fmt::Write` for serial Tx

### Breaking
    - LSI is no longer enabled by default, requires `lsi(true)` when configuring the rcc 

## [v0.3.4] - 2018-12-31

### Fixed
    - Hardcoded stm32 device crate in `i2c`

### Added
    - TSC clear flag API

## [v0.3.3] - 2018-12-12

### Fixed
    - Bumped stm32l4 to v0.5.0
    - Fixed timer start macro

## [v0.3.2] - 2018-12-12

### Added
    - DMA buffers now use as-slice

### Fixed
    - This crate now compiles on stable (1.31)!

## [v0.3.1] - 2018-12-10

### Added
    - I2C support using embedded-hal traits
    - Added GPIO input embedded-hal traits

## [v0.3.0] - 2018-11-21

### Breaking
    - Move device selection (l4x1, l4x2, etc.) behind feature, in line with stm32f4xx-hal.
    - Change crate name accordingly to `stm32l4-hal`.

### Fixed
    - Update crate dependencies.

## [v0.2.7] - 2018-11-21

### Breaking
    - EOL the stm32l432xx-hal crate

## [v0.2.6] - 2018-11-03

## Fixed

    - TSC flags not getting cleared by the `aquire` tsc method.

## [v0.2.5] - YANKED

## [v0.2.4] - 2018-10-32

### Fixed
    - Updated examples and bumped dependancies.

## [v0.2.3] - 2018-10-06

### Fixed
    - RCC configuration for higher clocks, using the wrong multiplyier reg caused issue with time sensative peripherals

### Added
    - New TSC Configuration, to set the prescaler and max count value.

## [v0.2.2] - 2018-08-25
    - Fixed documentation link in readme.

## [v0.2.1] - 2018-08-25

### Added
    - Timer implementations for timer15 & timer 16
    - Two new TSC API's, `read` & `read_unchecked`
        - `read` checks the supplied pin is the pin we are reading
        - `read_unchecked` returns the contents of the count register

### Fixed
    - Channel pins do not require Schmitt trigger hysteresis

## [v0.2.0] - 2018-08-21

### Breaking
    - GPIO type signature changed to allow pin state and alterante function to be encoded in the pin type.

### Added
    - Touch sense controller peripheral support
        - Easy to use blocking API with interrupt support too

## [v0.1.1] - 2018-08-13

### Added
    - Idle line detection for Usart Rx
    - Idle Line Interrupt listening

### Fixed
    - DMA Circular peeking, now correctly resets `consumed_offset`

## v0.1.0 - 2018-08-05

- Initial release

[v0.7.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.6...v0.4.0
[v0.3.6]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.5...v0.3.6
[v0.3.5]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.4...v0.3.5
[v0.3.4]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.3...v0.3.4
[v0.3.3]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.2...v0.3.3
[v0.3.2]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.1...v0.3.2
[v0.3.1]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.3.0...v0.3.1
[v0.3.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.7...v0.3.0
[v0.2.7]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.6...v0.2.7
[v0.2.6]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.5...v0.2.6
[v0.2.5]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.4...v0.2.5
[v0.2.4]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.3...v0.2.4
[v0.2.3]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.2...v0.2.3
[v0.2.2]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.1...v0.2.2
[v0.2.1]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.1.1...v0.2.0
[v0.1.1]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.1.0...v0.1.1
[v0.1.0]: https://github.com/stm32-rs/stm32l4xx-hal/tree/v0.1.0
