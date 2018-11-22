# Change Log

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

## [v0.3.0] - 2018-11-21

### Breaking
    - Move device selection (l4x1, l4x2, etc.) behind feature, in line with stm32f4xx-hal.
    - Change crate name accordingly to `stm32l4-hal`.

### Fixed
    - Update crate dependencies.

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

[Unreleased]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.6...HEAD
[v0.2.6]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.5...v0.2.6
[v0.2.5]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.4...v0.2.5
[v0.2.4]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.3...v0.2.4
[v0.2.3]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.2...v0.2.3
[v0.2.2]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.1...v0.2.2
[v0.2.1]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.1.1...v0.2.0
[v0.1.1]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.1.0...v0.1.1
[v0.1.0]: https://github.com/MabezDev/stm32l432xx-hal/tree/v0.1.0
