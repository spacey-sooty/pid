trait PIDController {
    fn calculate(&mut self, position: f32, goal: f32) -> f32;
}

struct PIDConfig {
    p: f32,
    i: f32,
    d: f32,
    last_position: f32,
    integral: f32,
}

impl PIDController for PIDConfig {
    fn calculate(&mut self, position: f32, goal: f32) -> f32 {
        let error = goal - position;
        let deriv = self.last_position - position;
        self.last_position = position;
        self.integral += error;
        return (self.p * error) + (self.i * self.integral) - (self.d * deriv);
    }
}
