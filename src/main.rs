#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(lang_items)]
#![feature(asm_experimental_arch)]
#![feature(abi_avr_interrupt)]

use core::{arch::asm, panic::PanicInfo};

use atmega_hal::Peripherals;
use atmega_usbd::UsbBus;
use usb_device::device::{UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    fn delay() {
        // Hand-rolled 100ms delay - the avr-hal delay is acting strangely
        unsafe {
            asm!(
                "ldi r25, 0xff",
                "2:",
                "ldi r26, 0xff",
                "1:",
                "subi r26, 1",
                "brne 1b",
                "subi r25, 1",
                "brne 2b",
                out("r25") _,
                out("r26") _,
            )
        }
    }
    let dp = unsafe { Peripherals::steal() };
    let pins = atmega_hal::pins!(dp);
    let mut status = pins.pe6.into_output();
    loop {
        status.set_high();
        delay();
        status.set_low();
        delay();
        status.set_high();
        delay();
        delay();
        delay();
        status.set_low();
        delay();
        delay();
        delay();
        delay();
        delay();
    }
}

#[lang = "eh_personality"]
#[no_mangle]
pub unsafe extern "C" fn rust_eh_personality() -> () {}

#[no_mangle]
#[cfg(not(test))]
pub extern "C" fn main() {
    main_inner();
}

fn main_inner() {
    let dp = Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let mut status = pins.pe6.into_output();
    let mut indicator = pins.pb7.into_output();

    // Configure PLL interface
    // prescale 16MHz crystal -> 8MHz
    pll.pllcsr.write(|w| w.pindiv().set_bit());
    // 96MHz PLL output; /1.5 for 64MHz timers, /2 for 48MHz USB
    pll.pllfrq
        .write(|w| w.pdiv().mhz96().plltm().factor_15().pllusb().set_bit());

    // Enable PLL
    pll.pllcsr.modify(|_, w| w.plle().set_bit());

    // Check PLL lock
    while pll.pllcsr.read().plock().bit_is_clear() {}

    status.set_high();

    let usb_bus = UsbBus::new(usb);
    let mut hid_class = HIDClass::new(&usb_bus, KeyboardReport::desc(), 1);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("Foo")
        .product("Bar")
        .device_class(0xff)
        .max_power(500)
        .build();

    status.set_high();

    // let mut next_char = 0;

    loop {
        if usb_device.poll(&mut [&mut hid_class]) {
            // let report = if let Some(c) = PAYLOAD.get(next_char) {
            //     next_char += 1;
            //     ascii_to_keycode(*c)
            // } else {
            //     indicator.set_high();
            //     KeyboardReport {
            //         modifier: 0,
            //         reserved: 0,
            //         leds: 0,
            //         keycodes: [0; 6],
            //     }
            // };
            // hid_class.push_input(&report);
        }
    }
}

// const PAYLOAD: &[u8] = b"Hello World!";

// fn ascii_to_keycode(b: u8) -> KeyboardReport {
//     let (shift, keycode) = if b.is_ascii_alphabetic() {
//         (b.is_ascii_uppercase(), b.to_ascii_lowercase() - b'a' + 0x04)
//     } else {
//         match b {
//             b' ' => (false, 0x2c),
//             b'!' => (true, 0x1e),
//             _ => unimplemented!(),
//         }
//     };
//     KeyboardReport {
//         modifier: if shift { 2 } else { 0 },
//         reserved: 0,
//         leds: 0,
//         keycodes: [keycode, 0, 0, 0, 0, 0],
//     }
// }
