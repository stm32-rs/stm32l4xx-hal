//! The feature detection module provides a level of abstraction and (hopefully) clarity to the crate features defined in cargo.toml
//! Primarily this will take the form of macros wrapping #[cfg(...)], but some const declarations may also prove useful
//!
//! NOTE: macros are disallowed inside #[cfg(...)] so the macro calls themselves will have to enclose this

mod peripherals;
pub(crate) use peripherals::*;
