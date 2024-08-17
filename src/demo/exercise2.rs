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
use embassy_rp::pwm::{Config as ConfigPwm, Pwm}; // PWM config
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

    let mut config_pwm: ConfigPwm = Default::default();
    config_pwm.top = 0x8000;
    config_pwm.compare_a = config_pwm.top;
    config_pwm.compare_b = config_pwm.top;

    let mut rgb_1_2_pin = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, config_pwm.clone());
    let mut rgb_3_pin = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, config_pwm.clone());

    let delay = Duration::from_secs(1);
    loop {
        if config_pwm.compare_a < config_pwm.top / 10 || config_pwm.compare_b < config_pwm.top / 10
        {
            config_pwm.compare_a = config_pwm.top + config_pwm.top / 10;
            config_pwm.compare_b = config_pwm.top + config_pwm.top / 10;
        }
        Timer::after(delay).await;
        config_pwm.compare_a -= config_pwm.top / 10;
        config_pwm.compare_b -= config_pwm.top / 10;
        rgb_1_2_pin.set_config(&config_pwm);
        rgb_3_pin.set_config(&config_pwm);
    }
}
