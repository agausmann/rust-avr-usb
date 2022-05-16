#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(lang_items)]
#![feature(abi_avr_interrupt)]

use core::panic::PanicInfo;

use arduino_hal::{delay_ms, pins, Peripherals};
use atmega_usbd::UsbBus;
use usb_device::device::{UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let dp = unsafe { Peripherals::steal() };
    let pins = pins!(dp);
    let mut status = pins.d13.into_output();
    loop {
        status.set_high();
        delay_ms(100);
        status.set_low();
        delay_ms(100);
        status.set_high();
        delay_ms(300);
        status.set_low();
        delay_ms(500);
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
    let pins = pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let mut status = pins.d13.into_output();

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

    let usb_bus = UsbBus::new(usb);
    let mut hid_class = HIDClass::new(&usb_bus, KeyboardReport::desc(), 1);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("Foo")
        .product("Bar")
        .device_class(0xef)
        .build();

    let mut report_buf = [0u8; 1];

    loop {
        if usb_device.poll(&mut [&mut hid_class]) {
            if hid_class.pull_raw_output(&mut report_buf).is_ok() {
                if report_buf[0] & 2 != 0 {
                    status.set_high();
                } else {
                    status.set_low();
                }
            }
        }
    }
}
