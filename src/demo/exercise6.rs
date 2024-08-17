#![no_std]
#![no_main]

mod eeprom_driver;
mod rgb_led;

use core::cell::RefCell;

use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::{info, *};
use display_interface_spi::{SPIInterface, SPIInterfaceNoCS};
use eeprom_driver::DriverAT24C256;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select4};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2CInterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, spi, Peripheral};
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_1::spi::SpiDevice;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::{Error, I2c as _};
use rgb_led::RgbLed;
use st7789::ST7789;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

async fn config_wifi(
    spawner: &Spawner,
    pin_23: PIN_23,
    pin_25: PIN_25,
    pio0: PIO0,
    pin_24: PIN_24,
    pin_29: PIN_29,
    dma_ch0: DMA_CH0,
) -> Control {
    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        pin_24,
        pin_29,
        dma_ch0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    control
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut control = config_wifi(
        &spawner, p.PIN_23, p.PIN_25, p.PIO0, p.PIN_24, p.PIN_29, p.DMA_CH0,
    )
    .await;

    // let mut rgb_controller = RgbLed::new(
    //     0x8000,
    //     p.PIN_0,
    //     p.PIN_1,
    //     p.PIN_2,
    //     p.PWM_SLICE0,
    //     p.PWM_SLICE1,
    // );

    let sda = p.PIN_20;
    let scl = p.PIN_21;

    let mut rom_controller = DriverAT24C256::new(p.I2C0, scl, sda, Irqs);

    // let mut button_a = Input::new(p.PIN_12, Pull::Up);
    // let mut button_b = Input::new(p.PIN_13, Pull::Up);
    // let mut button_x = Input::new(p.PIN_14, Pull::Up);
    // let mut button_y = Input::new(p.PIN_15, Pull::Up);

    let mut config = spi::Config::default();
    config.frequency = 2_000_000;
    config.phase = spi::Phase::CaptureOnFirstTransition;
    config.polarity = spi::Polarity::IdleHigh;

    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;

    let spi = Spi::new_blocking(p.SPI0, clk, mosi, miso, config);

    let mut cs = Output::new(p.PIN_17, Level::High);
    let rst = Output::new(p.PIN_0, Level::Low);

    let spi_bus = Mutex::new(RefCell::new(spi));

    let device_spi = SpiDeviceWithConfig::new(&spi_bus, cs, config);

    let mut display = ST7789::new(di, rst, 240, 240);
    display.init(&mut embassy_time::Delay).unwrap();
    display
        .set_orientation(st7789::Orientation::Landscape)
        .unwrap();

    let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::new("IP workshop", Point::new(100, 100), character_style);

    loop {
        Timer::after_secs(3).await;
    }
}
