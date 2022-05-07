#![no_std]
#![cfg_attr(not(test), no_main)]

#[no_mangle]
#[cfg(not(test))]
pub extern "C" fn main() {
    main_inner();
}

use atmega_hal::Peripherals;

fn main_inner() {
    let peripherals = Peripherals::take().unwrap();
    loop {}
}
