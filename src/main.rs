#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(lang_items)]
#![feature(asm_experimental_arch)]

use core::{arch::asm, cell::Cell, panic::PanicInfo};

use atmega_hal::{
    pac::{
        usb_device::{udint, ueintx, UDINT, UEINTX},
        USB_DEVICE,
    },
    port::{mode::Output, Pin},
    Peripherals,
};
use avr_device::interrupt::{free as interrupt_free, CriticalSection, Mutex};
use usb_device::{
    bus::PollResult,
    class_prelude::{EndpointAddress, EndpointType, UsbBus, UsbBusAllocator},
    device::{UsbDeviceBuilder, UsbVidPid},
    UsbDirection, UsbError,
};
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

static mut INDICATOR: Option<Pin<Output>> = None;

fn trace() {
    unsafe {
        if let Some(ind) = &mut INDICATOR {
            ind.set_high();
        }
    }
}

fn main_inner() {
    let dp = Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let mut status = pins.pe6.into_output();
    let indicator = pins.pb7.into_output();

    unsafe { INDICATOR = Some(indicator.downgrade()) };

    usb.usbcon.reset();
    usb.uhwcon.reset();
    usb.udcon.reset();
    usb.udien.reset();

    // Power-On USB pads regulator
    usb.uhwcon.modify(|_, w| w.uvrege().set_bit());
    usb.usbcon.modify(|_, w| w.otgpade().set_bit());

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

    let usb_bus = Usb::new(usb);

    // let mut hid_class = HIDClass::new(&usb_bus, KeyboardReport::desc(), 1);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xf667, 0x2012))
        .manufacturer("Foo")
        .product("Bar")
        .device_class(0xef)
        .max_power(500)
        .build();

    status.set_high();

    loop {
        usb_device.poll(&mut []);
    }
}

const MAX_ENDPOINT: u8 = 6;

struct Usb {
    usb: Mutex<USB_DEVICE>,
    last_endpoint: u8,
    control_endpoints: u8,
    pending_ins: Mutex<Cell<u8>>,
    is_suspended: Mutex<Cell<bool>>,
}

impl Usb {
    fn new(usb: USB_DEVICE) -> UsbBusAllocator<Self> {
        // NB: FRZCLK cannot be configured at the same time as USB
        usb.usbcon.modify(|_, w| w.usbe().set_bit());
        usb.usbcon.modify(|_, w| w.frzclk().clear_bit());

        UsbBusAllocator::new(Self {
            usb: Mutex::new(usb),
            last_endpoint: 0,
            control_endpoints: 0,
            pending_ins: Mutex::new(Cell::new(0)),
            is_suspended: Mutex::new(Cell::new(false)),
        })
    }

    fn set_current_endpoint(
        &self,
        cs: &CriticalSection,
        addr: EndpointAddress,
    ) -> usb_device::Result<()> {
        let addr = addr.index();
        if addr > self.last_endpoint as usize {
            return Err(UsbError::InvalidEndpoint);
        }
        self.usb
            .borrow(cs)
            .uenum
            .write(|w| unsafe { w.bits(addr as u8) });
        Ok(())
    }

    fn is_control_endpoint(&self, addr: EndpointAddress) -> bool {
        self.control_endpoints & (1 << addr.index()) != 0
    }

    fn endpoint_buffer_size(&self, cs: &CriticalSection) -> u16 {
        let usb = self.usb.borrow(cs);
        // 0 => 2^3, 1 => 2^4, etc
        let bits = usb.uecfg1x.read().epsize().bits() + 3;
        1 << bits
    }

    fn endpoint_byte_count(&self, cs: &CriticalSection) -> u16 {
        let usb = self.usb.borrow(cs);
        // Read high byte twice to attempt to mitigate race conditions?
        // The synchronization of these registers isn't really documented
        // anywhere, as far as I've seen.
        loop {
            let high_1 = usb.uebchx.read().bits();
            let low = usb.uebclx.read().bits();
            let high_2 = usb.uebchx.read().bits();
            if high_1 == high_2 {
                return ((high_1 as u16) << 8) | (low as u16);
            }
        }
    }
}

impl UsbBus for Usb {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> usb_device::Result<EndpointAddress> {
        // usb_device configures the control endpoint as both IN and OUT.
        // Both configurations are otherwise identical, so ignore the IN alloc:
        if ep_addr == Some(EndpointAddress::from_parts(0, UsbDirection::In)) {
            return Ok(ep_addr.unwrap());
        }

        // usb_device manually configures endpoint 0 internally.
        //   For all others, ignore the requested index.
        let ep_addr = match ep_addr {
            Some(addr) if addr.index() == 0 => addr,
            _ => {
                if self.last_endpoint >= MAX_ENDPOINT {
                    return Err(UsbError::EndpointOverflow);
                }
                self.last_endpoint += 1;
                EndpointAddress::from_parts(self.last_endpoint as usize, ep_dir)
            }
        };

        // Override specified direction for control endpoints;
        // they are all configured as "OUT" on the peripheral.
        let ep_dir = if ep_type == EndpointType::Control {
            UsbDirection::Out
        } else {
            ep_dir
        };

        let dir_cfg = match ep_dir {
            UsbDirection::In => true,
            UsbDirection::Out => false,
        };
        let type_cfg = match ep_type {
            EndpointType::Control => 0b00,
            EndpointType::Isochronous => 0b01,
            EndpointType::Bulk => 0b10,
            EndpointType::Interrupt => 0b11,
        };
        let size_cfg = match max_packet_size {
            0..=8 => 0b000,
            0..=16 => 0b001,
            0..=32 => 0b010,
            0..=64 => 0b011,
            0..=128 => 0b100,
            0..=256 => 0b101,
            0..=512 => 0b110,
            _ => return Err(UsbError::EndpointMemoryOverflow),
        };

        interrupt_free(|cs| {
            self.set_current_endpoint(cs, ep_addr)?;
            let usb = self.usb.borrow(cs);

            if ep_addr.index() != 0 && usb.ueconx.read().epen().bit_is_set() {
                return Err(UsbError::Unsupported);
            }
            usb.ueconx.write(|w| w.epen().set_bit());

            usb.uecfg0x
                .write(|w| w.epdir().bit(dir_cfg).eptype().bits(type_cfg));
            usb.uecfg1x
                .write(|w| w.epbk().bits(0).epsize().bits(size_cfg));
            usb.uecfg1x.modify(|_, w| w.alloc().set_bit());

            if usb.uesta0x.read().cfgok().bit_is_clear() {
                return Err(UsbError::Unsupported);
            }

            if ep_type == EndpointType::Control {
                self.control_endpoints |= 1 << ep_addr.index();
            }
            Ok(ep_addr)
        })
    }

    fn enable(&mut self) {
        interrupt_free(|cs| {
            self.usb
                .borrow(cs)
                .udcon
                .modify(|_, w| w.detach().clear_bit());
        })
    }

    fn reset(&self) {}

    fn set_device_address(&self, addr: u8) {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udaddr.modify(|_, w| w.uadd().bits(addr));
            usb.udaddr.modify(|_, w| w.adden().set_bit());
        });
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        interrupt_free(|cs| {
            self.set_current_endpoint(cs, ep_addr)?;
            let usb = self.usb.borrow(cs);

            if self.is_control_endpoint(ep_addr) {
                if usb.ueintx.read().txini().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }
                let buffer_size = self.endpoint_buffer_size(cs) as usize;
                if buf.len() > buffer_size {
                    return Err(UsbError::BufferOverflow);
                }
                for &byte in buf {
                    usb.uedatx.write(|w| unsafe { w.bits(byte) })
                }
                usb.ueintx.write_with_ones(|w| w.txini().clear_bit());
                Ok(buf.len())
            } else {
                if usb.ueintx.read().txini().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }

                //NB: rxouti/killbk needs to stay zero:
                usb.ueintx
                    .write_with_ones(|w| w.txini().clear_bit().rxouti().clear_bit());
                for &byte in buf {
                    if usb.ueintx.read().rwal().bit_is_clear() {
                        return Err(UsbError::BufferOverflow);
                    }
                    usb.uedatx.write(|w| unsafe { w.bits(byte) });
                }
                //NB: rxouti/killbk needs to stay zero:
                usb.ueintx
                    .write_with_ones(|w| w.fifocon().clear_bit().rxouti().clear_bit());

                let pending_ins = self.pending_ins.borrow(cs);
                pending_ins.set(pending_ins.get() | 1 << ep_addr.index());
                Ok(buf.len())
            }
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        interrupt_free(|cs| {
            self.set_current_endpoint(cs, ep_addr)?;
            let usb = self.usb.borrow(cs);

            if self.is_control_endpoint(ep_addr) {
                let ueintx = usb.ueintx.read();
                if ueintx.rxouti().bit_is_clear() && ueintx.rxstpi().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }
                let bytes_to_read = self.endpoint_byte_count(cs) as usize;
                if bytes_to_read > buf.len() {
                    return Err(UsbError::BufferOverflow);
                }
                for slot in &mut buf[..bytes_to_read] {
                    *slot = usb.uedatx.read().bits();
                }
                if ueintx.rxstpi().bit_is_set() {
                    usb.ueintx.write(|w| w.rxstpi().clear_bit());
                } else if ueintx.rxouti().bit_is_set() {
                    usb.ueintx.write(|w| w.rxouti().clear_bit());
                }
                usb.ueintx
                    .write(|w| w.rxouti().clear_bit().rxstpi().clear_bit());
                Ok(bytes_to_read)
            } else {
                if usb.ueintx.read().rxouti().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }

                usb.ueintx.write_with_ones(|w| w.rxouti().clear_bit());
                let mut bytes_read = 0;
                for slot in buf {
                    if usb.ueintx.read().rwal().bit_is_clear() {
                        break;
                    }
                    *slot = usb.uedatx.read().bits();
                    bytes_read += 1;
                }
                if usb.ueintx.read().rwal().bit_is_set() {
                    return Err(UsbError::BufferOverflow);
                }
                usb.ueintx.write_with_ones(|w| w.fifocon().clear_bit());
                Ok(bytes_read)
            }
        })
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt_free(|cs| {
            if self.set_current_endpoint(cs, ep_addr).is_ok() {
                let usb = self.usb.borrow(cs);
                usb.ueconx
                    .modify(|_, w| w.stallrq().bit(stalled).stallrqc().bit(!stalled));
            }
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt_free(|cs| {
            if self.set_current_endpoint(cs, ep_addr).is_ok() {
                let usb = self.usb.borrow(cs);
                usb.ueconx.read().stallrq().bit_is_set()
            } else {
                false
            }
        })
    }

    fn suspend(&self) {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udint.write_with_ones(|w| w.suspi().clear_bit());
            usb.usbcon.modify(|_, w| w.frzclk().set_bit());
            self.is_suspended.borrow(cs).set(true);
            //TODO disable PLL?
        });
    }

    fn resume(&self) {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);
            //TODO enable PLL?
            usb.usbcon.modify(|_, w| w.frzclk().clear_bit());
            usb.udint.write_with_ones(|w| w.wakeupi().clear_bit());
            self.is_suspended.borrow(cs).set(false);
        });
    }

    fn poll(&self) -> PollResult {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);

            let udint = usb.udint.read();
            let is_suspended = self.is_suspended.borrow(cs).get();
            if udint.suspi().bit_is_set() && !is_suspended {
                return PollResult::Suspend;
            }
            if udint.wakeupi().bit_is_set() && is_suspended {
                return PollResult::Resume;
            }

            let mut ep_out = 0u8;
            let mut ep_setup = 0u8;
            let mut ep_in_complete = 0u8;
            let pending_ins = self.pending_ins.borrow(cs);

            for ep in 0..=self.last_endpoint {
                let addr = EndpointAddress::from(ep);
                self.set_current_endpoint(cs, addr).unwrap();

                let ueintx = usb.ueintx.read();
                if ueintx.rxouti().bit_is_set() {
                    ep_out |= 1 << ep;
                }
                if ueintx.rxstpi().bit_is_set() {
                    ep_setup |= 1 << ep;
                }
                if pending_ins.get() & (1 << ep) != 0 && ueintx.txini().bit_is_set() {
                    ep_in_complete |= 1 << ep;
                    pending_ins.set(pending_ins.get() & !(1 << ep));
                }
            }
            if ep_out | ep_setup | ep_in_complete != 0 {
                return PollResult::Data {
                    ep_out: ep_out as u16,
                    ep_in_complete: ep_in_complete as u16,
                    ep_setup: ep_setup as u16,
                };
            }

            PollResult::None
        })
    }
}

/// Extension trait for conveniently clearing AVR interrupt flag registers.
///
/// To clear an interrupt flag, a zero bit must be written. However, there are
/// several other hazards to take into consideration:
///
/// 1. If you read-modify-write, it is possible that an interrupt flag will be
///   set by hardware in between the read and write, and writing the zero that
///   you previously read will clear that flag as well. So, use a default value
///   of all ones and specifically clear the bits you want. HOWEVER:
///
/// 2. Some bits of the interrupt flag register are reserved, and it is
///   specified that they should not be written as ones.
///
/// Implementers of this trait should provide an initial value to the callback
/// with all _known_ interrupt flags set to the value that has no effect (which
/// is 1, in most cases)
trait WriteWithOnes {
    type Writer;

    fn write_with_ones<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer;
}

impl WriteWithOnes for UDINT {
    type Writer = udint::W;

    fn write_with_ones<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer,
    {
        // Bits 1,7 reserved as do not set. Setting all other bits has no effect
        self.write(|w| f(unsafe { w.bits(0x7d) }))
    }
}

impl WriteWithOnes for UEINTX {
    type Writer = ueintx::W;

    fn write_with_ones<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer,
    {
        // Bit 5 read-only. Setting all other bits has no effect, EXCEPT:
        //  - RXOUTI/KILLBK should not be set for "IN" endpoints (XXX end-user beware)
        self.write(|w| f(unsafe { w.bits(0xdf) }))
    }
}
