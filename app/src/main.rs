#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use hal::{
    self,
    adc::{Adc, AdcDevice, AdcInterrupt, Align, InputType, SampleTime},
    clocks::Clocks,
    dma,
    dma::{Dma, DmaChannel, DmaInput, DmaInterrupt, DmaPeriph},
    pac,
    pac::{ADC1, DMA1},
    timer::TimerInterrupt,
};

use cortex_m;

static mut SPI_READ_BUF: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
const SPI_WRITE_BUF: [u8; 4] = [0x80, 0x20, 0x00, 0x00];

const I_CH1: u8 = 4;
const I_CH2: u8 = 15;
const VSENS: u8 = 3;

const SAMPLING_COUNT: usize = 3;
const ADC1_SEQUENCE: [u8; SAMPLING_COUNT] = [I_CH1, I_CH2, VSENS];

static mut ADC_READ_BUF: [u16; SAMPLING_COUNT] = [0; SAMPLING_COUNT];

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    // Import custom modules from tunepulse_rs crate
    use tunepulse_algo::{
        MotorController,
        motor_driver::{MotorType, PhasePattern, ControlMode}
    };

    use tunepulse_drivers::*;

    #[shared]
    struct Shared {
        spi1: encoder_spi::Spi1DMA,
        adc1: Adc<ADC1>,
    }

    #[local]
    struct Local {
        timer_pwm: pwm::TimPWM,
        underflow: bool,
        motor: MotorController,
        dma1: Dma<DMA1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let freq = 20000;
        let sysclk_freq = clock_cfg.sysclk(); // System clock frequency in Hz
        defmt::debug!("SYSTEM: Clock frequency is {} MHz", sysclk_freq / 1000000);
        init_driver_pins();

        let mut timer_pwm = pwm::TimPWM::new(dp.TIM2, &clock_cfg, freq);
        timer_pwm.begin();
        const MAX_SUP_VLTG: i32 = 69000;
        let motor = MotorController::new(MotorType::STEP, PhasePattern::ABCD, freq, MAX_SUP_VLTG);


        let spi1 = encoder_spi::Spi1DMA::new(dp.SPI1);

        let dma1 = Dma::new(dp.DMA1);
        dma::enable_mux1();
        dma::mux(DmaPeriph::Dma1, DmaChannel::C3, DmaInput::Spi1Tx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Spi1Rx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::Adc1);

        let mut adc1 = Adc::new_adc1(
            dp.ADC1,
            AdcDevice::One,
            Default::default(),
            clock_cfg.systick(),
        );

        for i in 0..SAMPLING_COUNT {
            adc1.set_sequence(ADC1_SEQUENCE[i], i as u8 + 1);
            adc1.set_input_type(ADC1_SEQUENCE[i], InputType::SingleEnded);
            adc1.set_sample_time(ADC1_SEQUENCE[i], SampleTime::T2);
        }
        adc1.set_sequence_len(SAMPLING_COUNT as u8);

        adc1.set_align(Align::Left);
        adc1.enable_interrupt(AdcInterrupt::EndOfSequence);

        (
            Shared { spi1, adc1 },
            Local {
                timer_pwm,
                underflow: true,
                motor,
                dma1,
            },
        )
    }

    fn init_driver_pins() {
        let mut dr_reset = pinout::driver::RESET.init();
        dr_reset.set_high();

        let mut dr_en = pinout::driver::ENABLE.init();
        dr_en.set_high();
    }

    #[task(binds = TIM2, shared = [spi1, adc1], local = [timer_pwm, underflow, motor])]
    fn tim2_period_elapsed(mut cx: tim2_period_elapsed::Context) {
        // Clear the update interrupt flag
        cx.local
            .timer_pwm
            .get_timer()
            .clear_interrupt(TimerInterrupt::Update);

        // Toggle the underflow flag
        *cx.local.underflow = !*cx.local.underflow;

        // Alternate between PWM and encoder reading
        if *cx.local.underflow {
            // Increment the tick counter, wrapping around on overflow
            // *cx.local.tick_counter = cx.local.tick_counter.wrapping_add(2);
            // Set the PWM duties on the timer
            cx.local.timer_pwm.apply_pwm(cx.local.motor.get_pwm());
            let adc_sup_voltage = unsafe { ADC_READ_BUF[2] >> 1 };

            let voltage = 1000;

            // Get encoder angle
            let pos: u16 = cx.shared.spi1.lock(|spi1| spi1.get_angle());

            // Calculate pwm states
            cx.local.motor.tick(voltage, pos, adc_sup_voltage);
        } else {
            // Start ADC DMA reading
            cx.shared.adc1.lock(|adc| {
                unsafe {
                    adc.read_dma(
                        &mut ADC_READ_BUF,
                        &ADC1_SEQUENCE,
                        DmaChannel::C1,
                        Default::default(),
                        DmaPeriph::Dma1,
                    )
                };
            });

            // Start SPI encoder read
            encoder_begin_read::spawn().expect("Failed to spawn encoder_begin_read");
        }
    }

    #[task(priority = 0, shared = [spi1])]
    async fn encoder_begin_read(mut cx: encoder_begin_read::Context) {
        cx.shared.spi1.lock(|spi1| unsafe {
            spi1.start();
            spi1.get_spi().transfer_dma(
                &SPI_WRITE_BUF,
                &mut SPI_READ_BUF,
                DmaChannel::C3,
                DmaChannel::C2,
                Default::default(),
                Default::default(),
                DmaPeriph::Dma1,
            );
        });
    }

    #[task(binds = DMA1_CH2, shared = [spi1], priority = 1)]
    fn encoder_end_read(mut cx: encoder_end_read::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C2,
            DmaInterrupt::TransferComplete,
        );
        cx.shared.spi1.lock(|spi1| {
            spi1.get_spi()
                .stop_dma(DmaChannel::C3, Some(DmaChannel::C2), DmaPeriph::Dma1);
            spi1.get_spi()
                .cleanup_dma(DmaPeriph::Dma1, DmaChannel::C3, Some(DmaChannel::C2));
            spi1.end(unsafe { SPI_READ_BUF });
        });
    }

    #[task(binds = DMA1_CH1, local = [dma1], shared = [adc1], priority = 1)]
    fn adc_end_read(cx: adc_end_read::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C1,
            DmaInterrupt::TransferComplete,
        );
        cx.local.dma1.stop(DmaChannel::C1);
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
