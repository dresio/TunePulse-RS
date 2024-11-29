// Float PID for comparison
pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    kff: f32,
    integral: f32,
    previous_error: f32,
    output: f32,
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32, kff: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            kff,
            integral: 0.0,
            previous_error: 0.0,
            output: 0.0,
        }
    }

    pub fn tick(&mut self, error: f32, feedfwd: f32, limit: f32) {
        // Calculate proportional term
        let p = self.kp * error;

        // Tustin's method for integrating the error with smoothing
        self.integral += (error + self.previous_error) * 0.5;

        // Clamp integral to avoid with anti-windup
        self.integral = self.integral.clamp(-limit, limit);

        // Calculate integral term
        let i = self.ki * self.integral;

        // Calculate derivative term
        let d = self.kd * (error - self.previous_error);

        // Update previous error for the next calculation
        self.previous_error = error;

        // Calculate the total output with clamping
        self.output = (p + i + d).clamp(-limit, limit);
    }

    pub fn output(&self) -> f32 {
        self.output
    }
}
