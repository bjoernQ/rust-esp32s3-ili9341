#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(feature = "fps")]
use core::cell::RefCell;
#[cfg(feature = "fps")]
use critical_section::Mutex;
#[cfg(feature = "fps")]
use esp_println::println;

use embedded_graphics::pixelcolor::Rgb565;
use esp_backtrace as _;

#[cfg(feature = "fps")]
use fugit::ExtU32;

#[cfg(feature = "fps")]
use esp_hal::{
    interrupt::{self, Priority},
    peripherals::{self},
    timer::systimer::{Alarm, Periodic, SystemTimer},
};

use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority},
    gpio::{Io, Level, Output},
    prelude::*,
    spi::{master::Spi, SpiMode},
};
use mipidsi::Builder;

mod spi_dma_displayinterface;

const WIDTH: usize = 300;
const HEIGHT: usize = 240;
const X_OFFSET: u16 = (320 - WIDTH as u16) / 2;
const Y_OFFSET: u16 = (240 - HEIGHT as u16) / 2;

const SINE_LUT: [u8; 512] = [
    127, 129, 131, 136, 138, 143, 145, 147, 151, 154, 158, 160, 162, 166, 169, 173, 175, 177, 181,
    183, 187, 189, 191, 195, 196, 200, 202, 204, 207, 209, 212, 214, 217, 219, 220, 223, 225, 227,
    229, 230, 233, 234, 236, 237, 239, 241, 242, 243, 244, 245, 247, 248, 249, 250, 250, 251, 252,
    253, 253, 253, 254, 254, 254, 254, 255, 254, 254, 254, 254, 254, 253, 253, 252, 252, 251, 250,
    250, 248, 248, 246, 245, 244, 243, 242, 240, 239, 237, 235, 234, 231, 230, 229, 226, 225, 222,
    220, 217, 216, 214, 211, 209, 205, 204, 202, 198, 196, 193, 191, 189, 185, 183, 179, 177, 175,
    171, 169, 164, 162, 160, 156, 154, 149, 147, 145, 140, 138, 134, 131, 127, 125, 123, 118, 116,
    111, 109, 107, 103, 100, 96, 94, 92, 88, 85, 81, 79, 77, 73, 71, 67, 65, 63, 59, 58, 54, 52,
    50, 47, 45, 42, 40, 37, 35, 34, 31, 29, 27, 25, 24, 21, 20, 18, 17, 15, 13, 12, 11, 10, 9, 7,
    6, 5, 4, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 6, 6, 8, 9, 10,
    11, 12, 14, 15, 17, 19, 20, 23, 24, 25, 28, 29, 32, 34, 37, 38, 40, 43, 45, 49, 50, 52, 56, 58,
    61, 63, 65, 69, 71, 75, 77, 79, 83, 85, 90, 92, 94, 98, 100, 105, 107, 109, 114, 116, 120, 123,
    127, 129, 131, 136, 138, 143, 145, 147, 151, 154, 158, 160, 162, 166, 169, 173, 175, 177, 181,
    183, 187, 189, 191, 195, 196, 200, 202, 204, 207, 209, 212, 214, 217, 219, 220, 223, 225, 227,
    229, 230, 233, 234, 236, 237, 239, 241, 242, 243, 244, 245, 247, 248, 249, 250, 250, 251, 252,
    253, 253, 253, 254, 254, 254, 254, 255, 254, 254, 254, 254, 254, 253, 253, 252, 252, 251, 250,
    250, 248, 248, 246, 245, 244, 243, 242, 240, 239, 237, 235, 234, 231, 230, 229, 226, 225, 222,
    220, 217, 216, 214, 211, 209, 205, 204, 202, 198, 196, 193, 191, 189, 185, 183, 179, 177, 175,
    171, 169, 164, 162, 160, 156, 154, 149, 147, 145, 140, 138, 134, 131, 127, 125, 123, 118, 116,
    111, 109, 107, 103, 100, 96, 94, 92, 88, 85, 81, 79, 77, 73, 71, 67, 65, 63, 59, 58, 54, 52,
    50, 47, 45, 42, 40, 37, 35, 34, 31, 29, 27, 25, 24, 21, 20, 18, 17, 15, 13, 12, 11, 10, 9, 7,
    6, 5, 4, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 6, 6, 8, 9, 10,
    11, 12, 14, 15, 17, 19, 20, 23, 24, 25, 28, 29, 32, 34, 37, 38, 40, 43, 45, 49, 50, 52, 56, 58,
    61, 63, 65, 69, 71, 75, 77, 79, 83, 85, 90, 92, 94, 98, 100, 105, 107, 109, 114, 116, 120, 123,
];

#[cfg(feature = "fps")]
static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, esp_hal::Blocking, 0>>>> =
    Mutex::new(RefCell::new(None));
#[cfg(feature = "fps")]
static mut FPS: u32 = 0;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    #[cfg(feature = "fps")]
    {
        let syst = SystemTimer::new(peripherals.SYSTIMER);
        let mut alarm0 = syst.alarm0.into_periodic();
        alarm0.set_period(1u32.secs());
        alarm0.set_interrupt_handler(systimer_target0);

        critical_section::with(|cs| {
            alarm0.enable_interrupt(true);
            ALARM0.borrow_ref_mut(cs).replace(alarm0);
        });
    }

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio7;
    let mosi = io.pins.gpio6;
    let cs = io.pins.gpio5;
    let miso = io.pins.gpio2;
    let dc = Output::new(io.pins.gpio4, Level::Low);
    let mut gpio_backlight = Output::new(io.pins.gpio45, Level::Low);
    let rst = Output::new(io.pins.gpio48, Level::Low);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi = Spi::new(peripherals.SPI2, 60u32.MHz(), SpiMode::Mode0)
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_miso(miso)
        .with_cs(cs)
        .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

    let mut delay = Delay::new();

    // gpio_backlight.set_low().unwrap();
    gpio_backlight.set_high();

    // create a DisplayInterface from SPI and DC pin, with no manual CS control
    let di = spi_dma_displayinterface::new_no_cs(WIDTH * HEIGHT * 2, spi, dc);

    // ESP32-S3-BOX display initialization workaround: Wait for the display to power up.
    // If delay is 250ms, picture will be fuzzy.
    // If there is no delay, display is blank
    delay.delay_millis(500u32);

    let mut display = Builder::ili9341_rgb565(di)
        .with_display_size(240, 320)
        .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .init(&mut delay, Some(rst))
        .unwrap();

    // just clear doesn't work?
    let mut count = 0;
    display
        .set_pixels(
            0,
            0,
            320,
            240,
            core::iter::from_fn(move || {
                count += 1;

                if count > 320 * 240 {
                    None
                } else {
                    Some(Rgb565::new(0, 0, 0))
                }
            }),
        )
        .unwrap();

    let mut i: usize = 0;
    let mut j: usize = 0;
    let mut k: usize = 0;
    loop {
        unsafe {
            for idx in (0..WIDTH * HEIGHT).step_by(2) {
                let y = idx / WIDTH;
                let x = idx % (WIDTH + 1);

                BUFFER[idx] = Rgb565::new(
                    (SINE_LUT[((x + SINE_LUT[(i + y) % 512] as usize) % 512) as usize] as u16
                        * 0b11111
                        / 255) as u8,
                    (SINE_LUT[((y + x + SINE_LUT[(x + j) % 512] as usize) % 512) as usize] as u32
                        * 0b111111
                        / 255) as u8,
                    (SINE_LUT[((x + k + SINE_LUT[(y + k) % 512] as usize) % 512) as usize] as u16
                        * 0b11111
                        / 255) as u8,
                );
            }
        }

        display
            .set_pixels(
                X_OFFSET,
                Y_OFFSET,
                WIDTH as u16 + X_OFFSET,
                HEIGHT as u16 + Y_OFFSET,
                unsafe { BUFFER.iter().cloned() },
            )
            .unwrap();

        i = i + 1;
        if i >= 255 {
            i = 0;
        }

        j = j + 2;
        if j >= 255 {
            j = 0;
        }

        k = k + 3;
        if k >= 255 {
            k = 0;
        }

        #[cfg(feature = "fps")]
        unsafe {
            FPS += 1;
        }
    }
}

static mut BUFFER: &mut [Rgb565; WIDTH * HEIGHT] = &mut [Rgb565::new(0, 0, 0); WIDTH * HEIGHT];

#[cfg(feature = "fps")]
#[handler]
fn systimer_target0() {
    println!("FPS {}", unsafe { FPS });
    unsafe {
        FPS = 0;
    }
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
