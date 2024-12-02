#![no_main]
#![no_std]

// use cortex_m as _;
use cortex_m_rt::entry;
use hal::pac;
use panic_halt as _;

use hal::{
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port, Pull},
    pac::TIM3,
    timer::{
        Alignment, CaptureCompareDma, CountDir, OutputCompare, TimChannel, Timer, TimerConfig,
        TimerInterrupt, UpdateReqSrc,
    },
};

use rtt_target::{rprint, rtt_init_print, ChannelMode::NoBlockSkip};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut led_green = Pin::new(Port::B, 14, PinMode::Output);

    let FREQUENCY: u16 = 10000;
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    /// Initialize the clocks for the microcontroller
    let clock_cfg = hal::clocks::Clocks::default();
    clock_cfg.setup().unwrap();

    // Create a new Timer with the specified frequency and configuration
    let mut timer_pwd = Timer::new_tim3(
        _dp.TIM3,
        FREQUENCY as f32,
        TimerConfig {
            one_pulse_mode: false,
            update_request_source: UpdateReqSrc::Any,
            auto_reload_preload: true,
            alignment: Alignment::Center1,
            capture_compare_dma: CaptureCompareDma::Update,
            direction: CountDir::Up,
        },
        &clock_cfg,
    );
    // Enable update interrupt for the timer
    timer_pwd.enable_interrupt(TimerInterrupt::Update);
    // Start the timer
    timer_pwd.enable();

    let mut counter = 0;
    let max_count = 1_000;

    loop {
        let duration = timer_pwd.get_timestamp_ms();
        rprint!("counter:{},{};", duration, counter);

        // Blink the green LED
        if counter >= max_count {
            led_green.toggle();
            counter = 0;
        }
        counter += 1;
    }
}

// #[panic_handler]
// fn panic() -> ! {
//     cortex_m::asm::udf()
// }
