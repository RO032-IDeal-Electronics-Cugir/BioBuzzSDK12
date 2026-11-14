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

    public enum TelemetryLevel {
        NONE,
        MATCH,
        DEBUG
    }

    public static double targetPower = 0.85;

    private DcMotor pollenMotor;
    private DcMotor nectarMotor;
    private Telemetry telemetry;
    private boolean isRunning = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            pollenMotor = hardwareMap.get(DcMotor.class, "pollenMotor");
            nectarMotor = hardwareMap.get(DcMotor.class, "nectarMotor");
            pollenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            nectarMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pollenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            nectarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            pollenMotor = null;
            nectarMotor = null;
        }
    }

    public void setPower(double power) {
        if (pollenMotor != null) {
            pollenMotor.setPower(power);
        }
        if (nectarMotor != null) {
            nectarMotor.setPower(power);
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

    public void updatePowerForDistance(double distanceInches) {
        double clampedDistance = Math.max(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_DISTANCE_INCHES,
                                 Math.min(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MAX_DISTANCE_INCHES, distanceInches));
        double ratio = (clampedDistance - org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_DISTANCE_INCHES) /
                       (org.firstinspires.ftc.teamcode.configurables.TurretConfig.MAX_DISTANCE_INCHES - org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_DISTANCE_INCHES);

        targetPower = org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_SHOOTER_POWER + ratio *
                      (org.firstinspires.ftc.teamcode.configurables.TurretConfig.MAX_SHOOTER_POWER - org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_SHOOTER_POWER);
    }

    public Command periodic() {
        return infinite(() -> {
            if (isRunning) {
                setPower(targetPower);
            } else {
                setPower(0.0);
            }
        });
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        telemetry.addData("Shooter Status", isRunning ? String.format(java.util.Locale.US, "ON (%.2f)", targetPower) : "OFF");

        if (level == TelemetryLevel.DEBUG) {
            telemetry.addData("Shooter Target Power", targetPower);
        }
    }
}
