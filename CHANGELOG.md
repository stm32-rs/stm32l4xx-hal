# Change Log

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

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

[Unreleased]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.1...HEAD
[v0.2.1]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.1.1...v0.2.0
[v0.1.1]: https://github.com/mabezdev/stm32l432xx-hal/compare/v0.1.0...v0.1.1
[v0.1.0]: https://github.com/MabezDev/stm32l432xx-hal/tree/v0.1.0
