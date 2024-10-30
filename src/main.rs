// filter_test.rs
// src/encoder/speed_estimator.rs

pub mod position_handling;
use position_handling::EncoderPositionHandler;


// Importing necessary Rust libraries for random number generation and graph plotting.
use plotters::prelude::*;
use rand::Rng;


// Main function to test the filter
fn main() {
    // Initializing the filter with an input of 0 and alpha value of 128 (0.5 in normalized terms)
    let mut pos_handler = EncoderPositionHandler::new(0, 1, 240);

    // Creating a vector to hold the filtered outputs
    let mut ideal = Vec::new();
    let mut inputs = Vec::new();
    let mut pos = Vec::new();

    // Creating a random number generator
    let mut rng = rand::thread_rng();

    // Running the filter for 2000 steps with a triangular wave input
    let mut input_value: u32 = 0;
    for i in 0..2000 {
        // Define the triangular wave input behavior
        if i % 1000 < 500 {
            input_value = input_value.wrapping_add(200);
        } else {
            input_value = input_value.wrapping_sub(200);
        }
        ideal.push(input_value as i32);

        // Adding random noise in the range of +/- 250
        let noise: i32 = rng.gen_range(-1000..=1000);
        let noisy_input = (input_value as i32 + noise).max(0) as u16;

        // Update the filter

        // Store the input and output values
        inputs.push(noisy_input as i32);
        pos.push(pos_handler.tick(noisy_input));
    }

    // Plotting the results using Plotters
    let root = BitMapBackend::new("position_filter_output.png", (2000, 3000)).into_drawing_area();
    root.fill(&RGBColor(31,31,31)).unwrap(); // Set the background to dark gray

    let mut chart = ChartBuilder::on(&root)
        .caption("Position Filter Output", ("sans-serif", 50))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0..2000, 0..100000_i32)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            (0..2000).zip(inputs.iter().cloned()),
            &BLUE,
        ))
        .unwrap()
        .label("Input Signal")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_series(LineSeries::new((0..2000).zip(ideal.iter().cloned()), &RED))
        .unwrap()
        .label("Ideal")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &RED));

    chart
        .draw_series(LineSeries::new((0..2000).zip(pos.iter().cloned()), &GREEN))
        .unwrap()
        .label("POS")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &GREEN));

    chart
        .configure_series_labels()
        .border_style(&WHITE)
        .draw()
        .unwrap();
}
