#![no_std]
#![no_main]

mod rgb_led;

use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select4};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, Peripheral};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use rgb_led::RgbLed;
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

    let mut rgb_controller = RgbLed::new(
        0x8000,
        p.PIN_0,
        p.PIN_1,
        p.PIN_2,
        p.PWM_SLICE0,
        p.PWM_SLICE1,
    );

    let mut button_a = Input::new(p.PIN_12, Pull::Up);
    let mut button_b = Input::new(p.PIN_13, Pull::Up);
    let mut button_x = Input::new(p.PIN_14, Pull::Up);
    let mut button_y = Input::new(p.PIN_15, Pull::Up);

    let delay = Duration::from_secs(1);

    let current_colors: [(u8, u8, u8); 3] = [(225, 235, 52), (31, 59, 171), (222, 22, 45)];
    let mut index = 0;

    loop {
        let selected = select4(
            button_a.wait_for_falling_edge(),
            button_b.wait_for_falling_edge(),
            button_x.wait_for_falling_edge(),
            button_y.wait_for_falling_edge(),
        )
        .await;

        match selected {
            embassy_futures::select::Either4::First(_) => {
                rgb_controller.set_rgb(
                    current_colors[0].0,
                    current_colors[0].1,
                    current_colors[0].2,
                );
            }
            embassy_futures::select::Either4::Second(_) => {
                rgb_controller.set_rgb(
                    current_colors[1].0,
                    current_colors[1].1,
                    current_colors[1].2,
                );
            }
            embassy_futures::select::Either4::Third(_) => {
                rgb_controller.set_rgb(
                    current_colors[2].0,
                    current_colors[2].1,
                    current_colors[2].2,
                );
            }
            embassy_futures::select::Either4::Fourth(_) => {
                let lerp_0_1 = RgbLed::lerp(current_colors[0], current_colors[1]);
                let lerp_1_2 = RgbLed::lerp(current_colors[1], current_colors[2]);

                rgb_controller.set_rgb(
                    current_colors[0].0,
                    current_colors[0].1,
                    current_colors[0].2,
                );
                Timer::after_secs(3).await;
                rgb_controller.set_rgb(lerp_0_1.0, lerp_0_1.1, lerp_0_1.2);
                Timer::after_secs(1).await;
                rgb_controller.set_rgb(
                    current_colors[1].0,
                    current_colors[1].1,
                    current_colors[1].2,
                );
                Timer::after_secs(3).await;
                rgb_controller.set_rgb(lerp_1_2.0, lerp_1_2.1, lerp_1_2.2);
                Timer::after_secs(1).await;
                rgb_controller.set_rgb(
                    current_colors[2].0,
                    current_colors[2].1,
                    current_colors[2].2,
                );
                Timer::after_secs(3).await;
                rgb_controller.set_color(0, 0, 0);
            }
        }

        Timer::after_millis(50).await;
    }
}
