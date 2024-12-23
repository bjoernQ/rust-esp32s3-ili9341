#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_graphics::pixelcolor::Rgb565;
use esp_backtrace as _;

use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority},
    gpio::{Level, Output},
    prelude::*,
    spi::master::Spi,
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

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init({
        let mut cfg = esp_hal::Config::default();
        cfg.cpu_clock = CpuClock::max();
        cfg
    });

    let sclk = peripherals.GPIO7;
    let mosi = peripherals.GPIO6;
    let cs = peripherals.GPIO5;
    let miso = peripherals.GPIO2;
    let dc = Output::new(peripherals.GPIO4, Level::Low);
    let mut gpio_backlight = Output::new(peripherals.GPIO45, Level::Low);
    let rst = Output::new(peripherals.GPIO48, Level::Low);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi = Spi::new_with_config(
        peripherals.SPI2,
        esp_hal::spi::master::Config {
            frequency: 60u32.MHz(),
            ..esp_hal::spi::master::Config::default()
        },
    )
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

    let mut display = Builder::new(mipidsi::models::ILI9341Rgb565, di)
        .display_size(240, 320)
        .orientation(mipidsi::options::Orientation::new())
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .reset_pin(rst)
        .init(&mut delay)
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
    }
}

static mut BUFFER: &mut [Rgb565; WIDTH * HEIGHT] = &mut [Rgb565::new(0, 0, 0); WIDTH * HEIGHT];
