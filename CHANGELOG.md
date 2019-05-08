# Change Log

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

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

[Unreleased]: https://github.com/stm32-rs/stm32l4xx-hal/compare/v0.4.0...HEAD
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
