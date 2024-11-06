// License and copyright information
// Copyright 2024 by CREAPUNK, http://creapunk.com
// Licensed under the Apache License, Version 2.0 (the "License");

use plotters::prelude::*;

use std::f64::consts::PI;
use std::process::Output;

mod encoder_position;

mod math_integer;
mod math_float;

mod analog;

mod phase_pattern_control;


// Основная функция для построения графика
fn main() {
    let target_value = 1000;
    let limit = 100;
    let num_iterations = 100;

    let mut pid_int = math_integer::controllers::pid::PID::new(25, 25, 25, 0);
    let mut pid_float = math_float::controllers::pid::PID::new(0.250, 0.25, 0.25, 0.0);
    let mut pid_ideal = math_float::controllers::pid::PID::new(0.250, 0.25, 0.25, 0.0);

    let mut float_integral_outputs = Vec::new();
    let mut int_integral_outputs = Vec::new();
    let mut ideal_integral_outputs = Vec::new();
    let mut ideal_integral_outputs_int = Vec::new();

    let mut float_integral = 0;
    let mut int_integral: i32 = 0;
    let mut ideal_integral = 0.0;

    for _ in 0..num_iterations {
        let float_error = target_value - float_integral;
        let int_error = (target_value - int_integral) as i16;
        let ideal_error = (target_value as f32 - ideal_integral);

        pid_float.tick(float_error as f32, 0.0, limit as f32);
        pid_int.tick(int_error, 0, limit as i16);
        pid_ideal.tick(ideal_error, 0.0, limit as f32);

        float_integral += pid_float.output() as i32;
        int_integral += pid_int.output() as i32;
        ideal_integral += pid_ideal.output();

        float_integral_outputs.push(float_integral as f64);
        int_integral_outputs.push(int_integral as f64);
        ideal_integral_outputs.push(ideal_integral as f64);
        ideal_integral_outputs_int.push(ideal_integral as i32 as f64);

        println!("Diff: {}", float_integral as i32 - int_integral as i32);
    }

    // Построение графика
    let root = BitMapBackend::new("pid_output_comparison.png", (1000, 2000)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption("PID Integral Output Comparison", ("sans-serif", 30))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0..num_iterations, 980.0..1030.0)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            (0..num_iterations).zip(float_integral_outputs.iter().cloned()),
            &RED,
        ))
        .unwrap()
        .label("Float PID Integral Output")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &RED));


        chart
        .draw_series(LineSeries::new(
            (0..num_iterations).zip(ideal_integral_outputs.iter().cloned()),
            &GREEN,
        ))
        .unwrap()
        .label("Ideal PID Output")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &GREEN));

        chart
        .draw_series(LineSeries::new(
            (0..num_iterations).zip(ideal_integral_outputs_int.iter().cloned()),
            &MAGENTA,
        ))
        .unwrap()
        .label("Float PID Integral Output as int")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &MAGENTA));

        chart
        .draw_series(LineSeries::new(
            (0..num_iterations).zip(int_integral_outputs.iter().cloned()),
            &BLUE,
        ))
        .unwrap()
        .label("Integer PID Integral Output")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &BLUE));


    chart
        .configure_series_labels()
        .border_style(&BLACK)
        .background_style(&WHITE.mix(0.8))
        .draw()
        .unwrap();
}
