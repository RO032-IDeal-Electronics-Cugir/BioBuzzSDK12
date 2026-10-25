package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
@Configurable
public class ShooterSubsystem {
    public static double targetPower = 0.85;

    private DcMotor shooterMotor;
    private Telemetry telemetry;
    private boolean isRunning = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            shooterMotor = null;
        }
    }

    public void setPower(double power) {
        if (shooterMotor != null) {
            shooterMotor.setPower(power);
        }
    }

    public void turnOn() {
        isRunning = true;
    }

    public void turnOff() {
        isRunning = false;
    }

    public void stop() {
        turnOff();
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void increasePower(double delta) {
        targetPower = Math.min(1.0, targetPower + delta);
    }

    public void decreasePower(double delta) {
        targetPower = Math.max(0.0, targetPower - delta);
    }

    public Command periodic() {
        return infinite(() -> {
            if (isRunning) {
                setPower(targetPower);
            } else {
                setPower(0.0);
            }

            if (telemetry != null) {
                telemetry.addData("Shooter Running", isRunning);
                telemetry.addData("Shooter Target Power", targetPower);
            }
        });
    }
}
