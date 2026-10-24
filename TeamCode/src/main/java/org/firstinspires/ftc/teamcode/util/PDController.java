package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDController {
    private double kP;
    private double kD;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PDController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
        timer.reset();
    }

    public double calculate(double error) {
        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) {
            dt = 0.02;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kD * derivative);
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
}
