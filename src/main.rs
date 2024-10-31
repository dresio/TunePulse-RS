// selector_motor_type.rs

// License and copyright information
// Copyright 2024 by CREAPUNK, http://creapunk.com
// Licensed under the Apache License, Version 2.0 (the "License");

use plotters::prelude::*;

use std::f32::consts::PI;

mod phase_pattern_control;
use phase_pattern_control::motor_selector::{MotorSelector, MotorType, VectorAxes2I16};
use phase_pattern_control::phase_selector::{PhaseSelector, PhasePattern};


fn main() {
    // Creating vectors for storing sine and cosine values in int16 range
    let mut sin_values = Vec::new();
    let mut cos_values = Vec::new();
    for i in 0..1000 {
        let angle = 2.0 * PI * (i as f32) / 100.0;
        sin_values.push((angle.sin() * i16::MAX as f32 / 8.0) as i16);
        cos_values.push((angle.cos() * i16::MAX as f32 / 8.0) as i16);
    }

    // Creating instance of SelectorMotorType
    let mut motor_selector = MotorSelector::new(
        MotorType::DC,
        VectorAxes2I16 {
            sin: sin_values[0],
            cos: cos_values[0],
        },
        5000,
        0,
    );

    let mut phase_selector = PhaseSelector::new(PhasePattern::ABCD, motor_selector.pwm_channels());

    // Running the motor control and storing PWM channel outputs
    let mut channel_a = Vec::new();
    let mut channel_b = Vec::new();
    let mut channel_c = Vec::new();
    let mut channel_d = Vec::new();
    let mut voltage = Vec::new();

    for i in 0..1000 {
        motor_selector.voltg = VectorAxes2I16 {
            sin: sin_values[i],
            cos: cos_values[i],
        };
        if i < 300 {
            if i > 150 {
                phase_selector.mode = PhasePattern::DCAB;
            }
        } else if i < 600 {
            motor_selector.mode = MotorType::STEPPER;
            phase_selector.mode = PhasePattern::ADBC;
            if i > 450 {
                phase_selector.mode = PhasePattern::ACDB;
            }
        } else {
            motor_selector.mode = MotorType::BLDC;
            motor_selector.voltg_sup += 10;
        }

        motor_selector.tick();
        phase_selector.ch_abcd = motor_selector.pwm_channels();
        phase_selector.tick();

        let pwm_channels = phase_selector.pwm_channels();
        voltage.push(motor_selector.voltg_sup as i32);
        channel_a.push(pwm_channels[0] as i32);
        channel_b.push(pwm_channels[1] as i32);
        channel_c.push(pwm_channels[2] as i32);
        channel_d.push(pwm_channels[3] as i32);
    }

    // Plotting the results using Plotters
    let root = BitMapBackend::new("volt_channels_output.png", (1500, 2000)).into_drawing_area();
    root.fill(&RGBColor(31, 31, 31)).unwrap(); // Set the background to dark gray

    let mut chart = ChartBuilder::on(&root)
        .caption("Voltage Channels Output", ("sans-serif", 50))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0..1000, -50..10000_i32)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            (0..1000).zip(channel_d.iter().cloned()),
            &MAGENTA,
        ))
        .unwrap()
        .label("Channel D")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &MAGENTA));

    chart
        .draw_series(LineSeries::new(
            (0..1000).zip(voltage.iter().cloned()),
            &MAGENTA,
        ))
        .unwrap()
        .label("VOLTAGE LIMIT")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &MAGENTA));

    chart
        .draw_series(LineSeries::new(
            (0..1000).zip(channel_a.iter().cloned()),
            &BLUE,
        ))
        .unwrap()
        .label("Channel A")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_series(LineSeries::new(
            (0..1000).zip(channel_b.iter().cloned()),
            &RED,
        ))
        .unwrap()
        .label("Channel B")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &RED));

    chart
        .draw_series(LineSeries::new(
            (0..1000).zip(channel_c.iter().cloned()),
            &GREEN,
        ))
        .unwrap()
        .label("Channel C")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &GREEN));

    chart
        .configure_series_labels()
        .border_style(&WHITE)
        .draw()
        .unwrap();
}
