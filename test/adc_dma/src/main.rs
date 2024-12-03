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
    pac::TIM3,
    pac::{ADC1, DMA1},
    timer::Timer,
    timer::TimerInterrupt,
};

const I_CH1: u8 = 4;
const I_CH2: u8 = 15;
const VSENS: u8 = 3;

const SAMPLING_COUNT: usize = 3;
const ADC1_SEQUENCE: [u8; SAMPLING_COUNT] = [I_CH1, I_CH2, VSENS];

static mut ADC_READ_BUF: [u16; SAMPLING_COUNT] = [0; SAMPLING_COUNT];

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        adc1: Adc<ADC1>,
    }

    #[local]
    struct Local {
        timer: Timer<TIM3>,
        dma1: Dma<DMA1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let mut adc = Adc::new_adc1(
            dp.ADC1,
            AdcDevice::One,
            Default::default(),
            clock_cfg.systick(),
        );

        for i in 0..SAMPLING_COUNT {
            adc.set_sequence(ADC1_SEQUENCE[i], i as u8 + 1);
            adc.set_input_type(ADC1_SEQUENCE[i], InputType::SingleEnded);
            adc.set_sample_time(ADC1_SEQUENCE[i], SampleTime::T2);
        }
        adc.set_sequence_len(SAMPLING_COUNT as u8);

        adc.set_align(Align::Left);
        adc.enable_interrupt(AdcInterrupt::EndOfSequence);

        let dma = Dma::new(dp.DMA1);
        dma::enable_mux1();
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::Adc1);

        let mut timer = Timer::new_tim3(dp.TIM3, 2., Default::default(), &clock_cfg);
        timer.enable_interrupt(TimerInterrupt::Update);
        timer.enable();


        (Shared { adc1: adc }, Local { timer, dma1: dma })
    }

    #[task(binds = DMA1_CH1, local = [dma1], shared = [adc1], priority = 1)]
    fn on_adc_dma_read(cx: on_adc_dma_read::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C1,
            DmaInterrupt::TransferComplete,
        );

        cx.local.dma1.stop(DmaChannel::C1);


        defmt::println!("ADC DMA read complete");

        let buf = unsafe { &mut ADC_READ_BUF };

        defmt::println!("voltage: {:?}", buf);
    }

    #[task(binds = TIM3, local = [timer], shared = [adc1], priority = 2)]
    fn on_timer(mut cx: on_timer::Context) {
        cx.local.timer.clear_interrupt(TimerInterrupt::Update);

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

    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
