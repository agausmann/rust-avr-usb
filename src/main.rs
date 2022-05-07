#![no_std]
#![cfg_attr(not(test), no_main)]

extern crate avr_std_stub;

#[no_mangle]
#[cfg(not(test))]
pub extern "C" fn main() {
    main_inner();
}

use core::cell::Cell;

use atmega_hal::{
    pac::{
        usb_device::{udint, ueintx, UDINT, UEINTX},
        USB_DEVICE,
    },
    Peripherals,
};
use avr_device::interrupt::{free as interrupt_free, CriticalSection, Mutex};
use usb_device::{
    bus::PollResult,
    class_prelude::{EndpointAddress, EndpointType, UsbBus, UsbBusAllocator},
    device::{UsbDeviceBuilder, UsbVidPid},
    UsbDirection, UsbError,
};

fn main_inner() {
    let dp = Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let mut status = pins.pe6.into_output();
    let mut indicator = pins.pb7.into_output();

    // Power-On USB pads regulator
    usb.uhwcon.modify(|_, w| w.uvrege().set_bit());

    // Configure PLL interface
    // prescale 16MHz crystal -> 8MHz
    pll.pllcsr.modify(|_, w| w.pindiv().set_bit());
    // 96MHz PLL output; /1.5 for 64MHz timers, /2 for 48MHz USB
    pll.pllfrq
        .modify(|_, w| w.pdiv().mhz96().plltm().factor_15().pllusb().set_bit());

    // Enable PLL
    pll.pllcsr.modify(|_, w| w.plle().set_bit());

    // Check PLL lock
    while pll.pllcsr.read().plock().bit_is_clear() {}

    let usb_bus = Usb::new(usb);
    let usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xf667, 0x2012))
        .manufacturer("Foo")
        .product("Bar")
        .max_power(500)
        .build();

    status.set_high();

    loop {
        usb_device.poll(&[]);
        if usb_device.state() == UsbDeviceState::Addressed {
            indicator.set_high();
        }
    }
}

const MAX_ENDPOINT: u8 = 6;

struct Usb {
    usb: Mutex<USB_DEVICE>,
    last_endpoint: u8,
    ep_in_complete: Mutex<Cell<u8>>,
}

impl Usb {
    fn new(usb: USB_DEVICE) -> UsbBusAllocator<Self> {
        usb.usbcon.modify(|_, w| w.usbe().set_bit());
        usb.udcon.modify(|_, w| w.lsm().clear_bit());

        UsbBusAllocator::new(Self {
            usb: Mutex::new(usb),
            last_endpoint: 0,
            ep_in_complete: Mutex::new(Cell::new(0)),
        })
    }

    fn set_current_endpoint(
        &self,
        cs: &CriticalSection,
        addr: EndpointAddress,
    ) -> usb_device::Result<()> {
        let addr = addr.index();
        if addr <= self.last_endpoint as usize {
            return Err(UsbError::InvalidEndpoint);
        }
        self.usb
            .borrow(cs)
            .uenum
            .write(|w| unsafe { w.bits(addr as u8) });
        Ok(())
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
            return Ok(());
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
                EndpointAddress::from_parts(self.last_endpoint, ep_dir)
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

            if usb.ueconx.read().epen().bit_is_set() {
                return Err(UsbError::Unsupported);
            }
            usb.ueconx.write(|w| w.epen().set_bit());

            usb.uecfg0x
                .write(|w| w.epdir().bit(dir_cfg).eptype().bits(type_cfg));
            usb.uecfg1x
                .write(|w| w.alloc().set_bit().epbk().bits(0).epsize().bits(size_cfg));

            if usb.uesta0x.read().cfgok().bit_is_set() {
                Ok(ep_addr)
            } else {
                Err(UsbError::Unsupported)
            }
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

            if usb.ueintx.read().txini().bit_is_clear() {
                return Err(UsbError::WouldBlock);
            }

            //NB: rxouti/killbk needs to stay zero:
            usb.ueintx
                .write_with_ones(|w| w.txini().clear_bit().rxouti().clear_bit());
            let mut bytes_written = 0;
            for &byte in buf {
                if usb.ueintx.read().rwal().bit_is_clear() {
                    return Err(UsbError::BufferOverflow);
                }
                usb.uedatx.write(|w| w.bits(byte));
                bytes_written += 1;
            }
            //NB: rxouti/killbk needs to stay zero:
            usb.ueintx
                .write_with_ones(|w| w.fifocon().clear_bit().rxouti().clear_bit());

            let ep_in_complete = self.ep_in_complete.borrow(cs);
            ep_in_complete.set(ep_in_complete.get() | 1 << u8::from(ep_addr));
            Ok(bytes_written)
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        interrupt_free(|cs| {
            self.set_current_endpoint(cs, ep_addr)?;
            let usb = self.usb.borrow(cs);

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
            //TODO disable PLL?
        });
    }

    fn resume(&self) {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);
            //TODO enable PLL?
            usb.usbcon.modify(|_, w| w.frzclk().clear_bit());
            usb.udint.write_with_ones(|w| w.wakeupi().clear_bit());
        });
    }

    fn poll(&self) -> PollResult {
        interrupt_free(|cs| {
            let usb = self.usb.borrow(cs);

            let udint = usb.udint.read();
            if udint.suspi().bit_is_set() {
                return PollResult::Suspend;
            }
            if udint.wakeupi().bit_is_set() {
                return PollResult::Resume;
            }

            let mut ep_out = 0u8;
            let mut ep_setup = 0u8;
            // AVR USB peripheral does not report when transfers are completed,
            // so fake it I guess?
            let ep_in_complete = self.ep_in_complete.borrow(cs).replace(0);

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
