use super::motor::{bldc, coil};
use super::MotorType;

const A: u32 = 1 << 0;
const B: u32 = 1 << 1;
const C: u32 = 1 << 2;
const D: u32 = 1 << 3;
const UNIPOLAR: u32 = 1 << 16;
const BIPOLAR: u32 = 0 << 16;

#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum Setup {
    // Bipolar probes
    BiAB = BIPOLAR | A | B,
    BiAC = BIPOLAR | A | C,
    BiAD = BIPOLAR | A | D,
    BiBC = BIPOLAR | B | C,
    BiBD = BIPOLAR | B | D,
    BiCD = BIPOLAR | C | D,
    BiABC = BIPOLAR | A | B | C,
    BiABD = BIPOLAR | A | B | D,
    BiACD = BIPOLAR | A | C | D,
    BiBCD = BIPOLAR | B | C | D,
    BiABCD = BIPOLAR | A | B | C | D,

    // Unipolar probes
    UniAB = UNIPOLAR | A | B,
    UniAC = UNIPOLAR | A | C,
    UniAD = UNIPOLAR | A | D,
    UniBC = UNIPOLAR | B | C,
    UniBD = UNIPOLAR | B | D,
    UniCD = UNIPOLAR | C | D,
    UniABC = UNIPOLAR | A | B | C,
    UniABD = UNIPOLAR | A | B | D,
    UniACD = UNIPOLAR | A | C | D,
    UniBCD = UNIPOLAR | B | C | D,
    UniABCD = UNIPOLAR | A | B | C | D,
}

#[derive(Debug)]
pub struct CurrentSenseAB<const PROBES: u32> {
    abcd_input: [i16; 4],
    ab_output: (i16, i16),
    motor_type: MotorType,
}

impl<const PROBES: u32> CurrentSenseAB<PROBES> {
    /// Конструктор
    pub fn new() -> Self {
        Self {
            abcd_input: [0; 4],
            ab_output: (i16::MIN, i16::MIN),
            motor_type: MotorType::UNDEFINED,
        }
    }

    /// Основной метод обработки
    pub fn tick(&mut self, currents: [i16; 4]) {
        self.abcd_input = currents;
        if is_bipolar(PROBES) {
            self.tick_bipolar();
        } else {
            self.tick_unipolar();
        }
    }

    /// Обработка для биполярного режима
    fn tick_bipolar(&mut self) {
        match probe_amount(PROBES) {
            1 => self.tick_bipolar_single(),
            2 => self.tick_bipolar_dual(),
            3 => self.tick_bipolar_triple(),
            4 => self.tick_bipolar_quad(),
            _ => self.ab_output = (i16::MIN, i16::MIN),
        }
    }

    /// Обработка для униполярного режима
    #[inline(always)]
    fn tick_unipolar(&mut self) {
        match probe_amount(PROBES) {
            4 => self.tick_unipolar_quad(),
            _ => self.ab_output = (i16::MIN, i16::MIN),
        }
    }

    /// Методы для биполярного режима
    #[inline(always)]
    fn tick_bipolar_single(&mut self) {
        self.ab_output = if let MotorType::DC = self.motor_type {
            (coil::current::single_bipolar(self.abcd_input[0]), 0)
        } else {
            (i16::MIN, i16::MIN)
        };
    }

    fn tick_bipolar_dual(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (i16::MIN, i16::MIN),
            MotorType::DC => (
                coil::current::dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEP => (
                coil::current::single_bipolar(self.abcd_input[0]),
                coil::current::single_bipolar(self.abcd_input[1]),
            ),
            MotorType::BLDC => bldc::current::dual(self.abcd_input[0], self.abcd_input[1]),
        };
    }

    fn tick_bipolar_triple(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                coil::current::dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEP => (
                coil::current::dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                coil::current::single_bipolar(self.abcd_input[2]),
            ),
            MotorType::BLDC => {
                bldc::current::triple(self.abcd_input[0], self.abcd_input[1], self.abcd_input[2])
            }
        };
    }

    fn tick_bipolar_quad(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                coil::current::dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEP => (
                coil::current::dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                coil::current::dual_bipolar(self.abcd_input[2], self.abcd_input[3]),
            ),
            MotorType::BLDC => {
                bldc::current::triple(self.abcd_input[0], self.abcd_input[1], self.abcd_input[2])
            }
        };
    }

    /// Методы для униполярного режима
    fn tick_unipolar_quad(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                coil::current::dual_unipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEP => (
                coil::current::dual_unipolar(self.abcd_input[0], self.abcd_input[1]),
                coil::current::dual_unipolar(self.abcd_input[2], self.abcd_input[3]),
            ),
            MotorType::BLDC => (0, 0),
        };
    }
}

const fn probe_amount(setup: u32) -> u32 {
    return (setup & 0b1111).count_ones();
}

const fn is_bipolar(setup: u32) -> bool {
    return (setup & 0b0000) != 0;
}
