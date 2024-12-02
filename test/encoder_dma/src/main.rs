#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use hal::dma;
use hal::dma::DmaInterrupt;
use hal::dma::{Dma, DmaChannel, DmaInput, DmaPeriph};
use hal::gpio::{Pin, PinMode, Port};
use hal::pac::SPI1;
use hal::spi::{BaudRate, Spi, SpiConfig, SpiMode};
use hal::timer::{Timer, TimerInterrupt};
use hal::{self, clocks::Clocks, pac, pac::TIM3};

use tunepulse_algo::encoder_position::EncoderPosition;

static mut SPI_READ_BUF: [u8; 4] = [0; 4];
static mut SPI_WRITE_BUF: [u8; 4] = [0x80, 0x20, 0x00, 0x00];

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use tunepulse_drivers::pinout::encoder;

    use super::*;

    #[shared]
    struct Shared {
        spi1: Spi<SPI1>,
    }

    #[local]
    struct Local {
        cs_pin: Pin,

        timer: Timer<TIM3>,

        encoder: EncoderPosition,
    }

    fn init_pins() {
        Pin::new(Port::A, 5, PinMode::Alt(5)); // PA5 SPI1_SCK
        Pin::new(Port::A, 6, PinMode::Alt(5)); // PA6 SPI1_MISO
        Pin::new(Port::A, 7, PinMode::Alt(5)); // PA7 SPI1_MOSI

        // Configure CS pin on Port C, Pin 4 as output
        let mut cs_pin = Pin::new(Port::C, 4, PinMode::Output);
        cs_pin.set_high();
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let freq = 2; // Hz
        let _cp = ctx.core;
        let dp = ctx.device;

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        init_pins();

        let spi_cfg = SpiConfig {
            mode: SpiMode::mode1(),
            ..Default::default()
        };

        let mut spi1 = Spi::new(dp.SPI1, spi_cfg, BaudRate::Div32);

        let _dma = Dma::new(dp.DMA1);
        dma::enable_mux1();
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::Spi1Tx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Spi1Rx);

        let mut encoder = EncoderPosition::new(0, freq, 128);

        let mut timer = Timer::new_tim3(dp.TIM3, freq as f32, Default::default(), &clock_cfg);
        timer.enable_interrupt(TimerInterrupt::Update);
        timer.enable();

        

        (
            Shared { spi1 },
            Local {
                cs_pin: Pin::new(Port::C, 4, PinMode::Output),
                timer,
                encoder,
            },
        )
    }

    #[task(binds = DMA1_CH2, local=[cs_pin, encoder], shared = [spi1], priority = 1)]
    fn on_encoder_rx(mut cx: on_encoder_rx::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C2,
            DmaInterrupt::TransferComplete,
        );

        defmt::println!("SPI DMA read complete");

        cx.shared.spi1.lock(|spi1| {
            spi1.stop_dma(DmaChannel::C1, Some(DmaChannel::C2), DmaPeriph::Dma1);
            spi1.cleanup_dma(DmaPeriph::Dma1, DmaChannel::C1, Some(DmaChannel::C2));
        });

        unsafe {
            let buf = unsafe { &mut SPI_READ_BUF };

            let respond = ((buf[2] as u16) << 8) | buf[3] as u16;
            let res = respond << 1;

            cx.local.encoder.tick(res);

            let pos =  cx.local.encoder.position();

            defmt::println!("Data read: {:?}", res);
            defmt::println!("Encoder position: {:?}", pos);
        }

        cx.local.cs_pin.set_high();
    }

    #[task(binds = TIM3, local=[timer], shared = [spi1], priority = 2)]
    fn on_timer(mut cx: on_timer::Context) {
        cx.local.timer.clear_interrupt(TimerInterrupt::Update);
        req_spi_transfer::spawn().expect("can not spawn on_spi_transfer");
    }

    #[task(priority = 0, shared = [spi1])]
    async fn req_spi_transfer(mut cx: req_spi_transfer::Context) {
        defmt::println!("transfer_dma");

        Pin::new(Port::C, 4, PinMode::Output).set_low();

        cx.shared.spi1.lock(|spi1| unsafe {
            spi1.transfer_dma(
                &SPI_WRITE_BUF,
                &mut SPI_READ_BUF,
                DmaChannel::C1,
                DmaChannel::C2,
                Default::default(),
                Default::default(),
                DmaPeriph::Dma1,
            );
        });
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}