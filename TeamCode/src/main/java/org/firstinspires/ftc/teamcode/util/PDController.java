package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDController {
    private double kP;
    private double kD;
    private double kF = 0.08; // Minimum feedforward power to overcome static friction
    private double tolerance = 3.0; // Default deadband tolerance in degrees
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PDController(double kP, double kD) {
        this(kP, kD, 0.08, 3.0);
    }

    public PDController(double kP, double kD, double kF) {
        this(kP, kD, kF, 3.0);
    }

    public PDController(double kP, double kD, double kF, double tolerance) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        timer.reset();
    }

    public double calculate(double error) {
        return calculate(error, tolerance);
    }

    public double calculate(double error, double tolerance) {
        this.tolerance = tolerance;

        // If error is within the deadband tolerance, stop turning and clear state to prevent jitter
        if (Math.abs(error) <= tolerance) {
            lastError = 0;
            return 0.0;
        }

        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) {
            dt = 0.02;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kD * derivative);

        // Add static friction feedforward if error is outside tolerance
        if (Math.abs(output) > 0.001) {
            output += Math.signum(output) * kF;
        }

        // Clamp output to [-1.0, 1.0]
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public boolean isWithinTolerance(double error) {
        return Math.abs(error) <= tolerance;
    }

    public void reset() {
        lastError = 0;
        timer.reset();
    }

    public void setKp(double kP) {
        this.kP = kP;
    }

    public void setKd(double kD) {
        this.kD = kD;
    }

    public void setKf(double kF) {
        this.kF = kF;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getTolerance() {
        return tolerance;
    }
}
