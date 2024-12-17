#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use hal::dma;
use hal::dma::DmaInterrupt;
use hal::dma::{Dma, DmaChannel, DmaInput, DmaPeriph};
use hal::timer::{Timer, TimerInterrupt};
use hal::{self, clocks::Clocks, pac, pac::TIM3};

use tunepulse_algo::math_integer::motion::position_integrator::Position;
use tunepulse_drivers::encoder_spi;

static mut SPI_READ_BUF: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
static mut SPI_WRITE_BUF: [u8; 4] = [0x80, 0x20, 0x00, 0x00];

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        spi1: encoder_spi::Spi1DMA,
    }

    #[local]
    struct Local {
        timer: Timer<TIM3>,

        encoder: Position,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let freq = 1; // Hz
        let _cp = ctx.core;
        let dp = ctx.device;

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let mut spi1 = encoder_spi::Spi1DMA::new(dp.SPI1);

        let _dma = Dma::new(dp.DMA1);
        dma::enable_mux1();
        dma::mux(DmaPeriph::Dma1, DmaChannel::C3, DmaInput::Spi1Tx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Spi1Rx);

        let mut encoder = Position::new();

        let mut timer = Timer::new_tim3(dp.TIM3, freq as f32, Default::default(), &clock_cfg);
        timer.enable_interrupt(TimerInterrupt::Update);
        timer.enable();

        (Shared { spi1 }, Local { timer, encoder })
    }

    #[task(binds = TIM3, local=[timer], shared = [spi1], priority = 2)]
    fn on_timer( cx: on_timer::Context) {
        cx.local.timer.clear_interrupt(TimerInterrupt::Update);
        encoder_begin_read::spawn().expect("can not spawn on_spi_transfer");
    }

    #[task(priority = 0, shared = [spi1])]
    async fn encoder_begin_read(mut cx: encoder_begin_read::Context) {
        defmt::println!("transfer_dma");

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

    #[task(binds = DMA1_CH2, local=[encoder], shared = [spi1], priority = 1)]
    fn encoder_end_read(mut cx: encoder_end_read::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C2,
            DmaInterrupt::TransferComplete,
        );

        defmt::println!("SPI DMA read complete");

        let mut res: u16 = 0;
        cx.shared.spi1.lock(|spi1| {
            spi1.get_spi()
                .stop_dma(DmaChannel::C3, Some(DmaChannel::C2), DmaPeriph::Dma1);
            spi1.get_spi()
                .cleanup_dma(DmaPeriph::Dma1, DmaChannel::C3, Some(DmaChannel::C2));
            res = spi1.end(unsafe { SPI_READ_BUF });
        });

        cx.local.encoder.tick(res);

        let pos = cx.local.encoder.position();

        defmt::println!("Data read: {:?}", res);
        defmt::println!("Encoder position: {:?}", pos);
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
