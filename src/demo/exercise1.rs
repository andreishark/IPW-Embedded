#![no_std]
#![no_main]

use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pwm::Config as ConfigPwm; // PWM config
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
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

    let mut green_pin = Output::new(p.PIN_4, Level::Low);
    let mut red_pin = Output::new(p.PIN_5, Level::Low);

    // let delay = Duration::from_secs(1);
    loop {
        green_pin.set_low();
        red_pin.set_high();
        Timer::after_secs(1).await;
        red_pin.set_low();
        green_pin.set_high();
        Timer::after_secs(1).await;

        // info!("led on!");
        // control.gpio_set(0, true).await;
        // Timer::after(delay).await;
        //
        // info!("led off!");
        // control.gpio_set(0, false).await;
        // Timer::after(delay).await;
    }
}
