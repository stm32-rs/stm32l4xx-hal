# Running examples
The easiest way to run examples is with the probe-run cargo extension.
See the [Installation](https://github.com/knurling-rs/probe-run#installation) section of the probe-run readme for details.

Examples will print messages using RTT for transport. Probe-run will open the debug log once it has completed flashing the firmware. Use probe-run --list-chips and find the correct ID for your MCU

Running the blinky example on a L433 Nucleo
```sh
# arguments after the "-- " are to tell probe-run which MCU we are flashing the image to
cargo run --features=rt,stm32l433 --example=blinky -- --chip STM32L433RCTx
```