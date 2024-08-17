#![no_std]
#![no_main]

mod eeprom_driver;
mod rgb_led;

use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::{info, *};
use eeprom_driver::DriverAT24C256;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select4};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2CInterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, Peripheral};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::{Error, I2c as _};
use rgb_led::RgbLed;
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

    let mut rgb_controller = RgbLed::new(
        0x8000,
        p.PIN_0,
        p.PIN_1,
        p.PIN_2,
        p.PWM_SLICE0,
        p.PWM_SLICE1,
    );

    let sda = p.PIN_20;
    let scl = p.PIN_21;

    let mut rom_controller = DriverAT24C256::new(p.I2C0, scl, sda, Irqs);

    // let mut button_a = Input::new(p.PIN_12, Pull::Up);
    // let mut button_b = Input::new(p.PIN_13, Pull::Up);
    // let mut button_x = Input::new(p.PIN_14, Pull::Up);
    // let mut button_y = Input::new(p.PIN_15, Pull::Up);

    let colors: [(u8, u8, u8); 10] = [
        (255, 0, 0),
        (125, 30, 40),
        (215, 55, 100),
        (0, 55, 100),
        (0, 12, 11),
        (0, 0, 0),
        (100, 13, 5),
        (22, 33, 11),
        (1, 2, 3),
        (97, 118, 139),
    ];
    rom_controller.write_colors(0x50, colors).await.unwrap();

    loop {
        let colors = rom_controller.read_colors::<10>(0x50).await.unwrap();
        for color in colors {
            rgb_controller.set_rgb(color.0, color.1, color.2);
            Timer::after_secs(2).await;
        }
        rgb_controller.set_color(0, 0, 0);
        Timer::after_secs(10).await;
    }
}
