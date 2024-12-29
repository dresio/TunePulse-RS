// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

pub struct BufferFIFO<T, const N: usize> {
    buffer: [T; N],

    idx: usize,
}

// Constants and methods used during calibration
impl<T, const N: usize> BufferFIFO<T, N>
where
    T: Default + Copy, 
{

    pub fn new() -> Self {
        Self {
            buffer: [T::default(); N], 
            idx: 0, 
        }
    }

    pub fn write(&mut self, value: T) {
        self.buffer[self.idx] = value;
        self.idx = (self.idx + 1) % N;
    }

    pub fn read(&self) -> T {
        self.buffer[self.idx]
    }

    pub fn pop(&mut self, value: T) -> T {
        let temp =self.buffer[self.idx];
        self.buffer[self.idx] = value;
        self.idx = (self.idx + 1) % N;
        temp
    }
}
