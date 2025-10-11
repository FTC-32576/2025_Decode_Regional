package ftc.team.Java_Is_AllMight.Control;

public class PIDController {

    private final PIDConfig config;
    private double integral;
    private double previousError;

    public PIDController(PIDConfig config) {
        this.config = config;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;

        if (Math.abs(error) < config.iZone) {
            integral += error;
        } else {
            integral = 0;
        }

        double derivative = error - previousError;
        previousError = error;

        return config.kP * error + config.kI * integral + config.kD * derivative + config.kF * setpoint;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
