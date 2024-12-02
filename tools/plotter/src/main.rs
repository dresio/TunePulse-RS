use egui::{CentralPanel, Color32, Plot, PlotPoints};
use egui_plotter::LinePlot;
use rtt_target::{rtt_init, rtt_read, rtt_write};
use std::{thread, time::Duration};

fn main() {
    // Initialize RTT (connect to the RTT channel)
    rtt_init!(); // Initializes RTT for receiving data

    // Create a vector to store encoder positions
    let mut encoder_positions: Vec<f64> = Vec::new();

    loop {
        // Try reading from the RTT channel
        if let Ok(data) = rtt_read() {
            // Parse the encoder position from the received data
            if let Some(position) = parse_encoder_position(&data) {
                encoder_positions.push(position);
                plot_encoder_data(&encoder_positions);
            }
        }

        // Sleep before reading again
        thread::sleep(Duration::from_millis(100));
    }
}

fn parse_encoder_position(data: &str) -> Option<f64> {
    // Try to parse the encoder position value from the string
    let parts: Vec<&str> = data.split(": ").collect();
    if parts.len() == 2 {
        parts[1].parse().ok()
    } else {
        None
    }
}

fn plot_encoder_data(encoder_positions: &[f64]) {
    // Plot the encoder position using egui
    let plot = Plot::new("Encoder Position Over Time").line(
        LinePlot::new(
            encoder_positions
                .iter()
                .enumerate()
                .map(|(i, &v)| [i as f64, v]),
        )
        .color(Color32::RED),
    );

    // Here you can integrate with egui to display the plot
    // Use an egui context to refresh the UI
    // context.request_repaint();
}
