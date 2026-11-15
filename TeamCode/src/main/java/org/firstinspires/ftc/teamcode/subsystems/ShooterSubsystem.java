package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    // Coeficienți Regresie Polinomială pentru Shooter POLLEN
    public static double POLLEN_POWER_A = 0.0;
    public static double POLLEN_POWER_B = 0.003;
    public static double POLLEN_POWER_C = 0.60;

    public static double POLLEN_PITCH_A = 0.0;
    public static double POLLEN_PITCH_B = 0.002;
    public static double POLLEN_PITCH_C = 0.40;

    // Coeficienți Regresie Polinomială pentru Shooter NECTAR
    public static double NECTAR_POWER_A = 0.0;
    public static double NECTAR_POWER_B = 0.003;
    public static double NECTAR_POWER_C = 0.60;

    public static double NECTAR_PITCH_A = 0.0;
    public static double NECTAR_PITCH_B = 0.002;
    public static double NECTAR_PITCH_C = 0.40;

    public static double targetPower = 0.85;

    private DcMotor pollenMotor;
    private DcMotor nectarMotor;
    private Servo pitchServoPollen;
    private Servo pitchServoNectar;

    private double pollenPower = 0.85;
    private double nectarPower = 0.85;
    private double pollenPitchPosition = 0.50;
    private double nectarPitchPosition = 0.50;

    private Telemetry telemetry;
    private boolean isRunning = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            pollenMotor = hardwareMap.get(DcMotor.class, "pollenMotor");
            pollenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pollenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            pollenMotor = null;
        }

        try {
            nectarMotor = hardwareMap.get(DcMotor.class, "nectarMotor");
            nectarMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            nectarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            nectarMotor = null;
        }

        try {
            pitchServoPollen = hardwareMap.get(Servo.class, "pitchServoPollen");
        } catch (Exception e) {
            pitchServoPollen = null;
        }

        try {
            pitchServoNectar = hardwareMap.get(Servo.class, "pitchServoNectar");
        } catch (Exception e) {
            pitchServoNectar = null;
        }
    }

    public void setPower(double power) {
        setPollenPower(power);
        setNectarPower(power);
    }

    public void setPollenPower(double power) {
        this.pollenPower = Math.max(0.0, Math.min(1.0, power));
        ShooterSubsystem.targetPower = this.pollenPower;
    }

    public void setNectarPower(double power) {
        this.nectarPower = Math.max(0.0, Math.min(1.0, power));
    }

    public double getPollenPower() {
        return pollenPower;
    }

    public double getNectarPower() {
        return nectarPower;
    }

    public void setPollenPitchPosition(double position) {
        this.pollenPitchPosition = Math.max(0.0, Math.min(1.0, position));
        if (pitchServoPollen != null) {
            pitchServoPollen.setPosition(this.pollenPitchPosition);
        }
    }

    public void setNectarPitchPosition(double position) {
        this.nectarPitchPosition = Math.max(0.0, Math.min(1.0, position));
        if (pitchServoNectar != null) {
            pitchServoNectar.setPosition(this.nectarPitchPosition);
        }
    }

    public double getPollenPitchPosition() {
        return pollenPitchPosition;
    }

    public double getNectarPitchPosition() {
        return nectarPitchPosition;
    }

    public void adjustPollenPower(double delta) {
        setPollenPower(pollenPower + delta);
    }

    public void adjustNectarPower(double delta) {
        setNectarPower(nectarPower + delta);
    }

    public void adjustPollenPitchPosition(double delta) {
        setPollenPitchPosition(pollenPitchPosition + delta);
    }

    public void adjustNectarPitchPosition(double delta) {
        setNectarPitchPosition(nectarPitchPosition + delta);
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
        adjustPollenPower(delta);
        adjustNectarPower(delta);
    }

    public void decreasePower(double delta) {
        adjustPollenPower(-delta);
        adjustNectarPower(-delta);
    }

    public void updatePowerForDistance(double distanceInches) {
        double d = Math.max(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_DISTANCE_INCHES,
                   Math.min(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MAX_DISTANCE_INCHES, distanceInches));

        // Calcul independent pentru Pollen
        double calculatedPollenPower = POLLEN_POWER_A * d * d + POLLEN_POWER_B * d + POLLEN_POWER_C;
        double calculatedPollenPitch = POLLEN_PITCH_A * d * d + POLLEN_PITCH_B * d + POLLEN_PITCH_C;

        // Calcul independent pentru Nectar
        double calculatedNectarPower = NECTAR_POWER_A * d * d + NECTAR_POWER_B * d + NECTAR_POWER_C;
        double calculatedNectarPitch = NECTAR_PITCH_A * d * d + NECTAR_PITCH_B * d + NECTAR_PITCH_C;

        setPollenPower(calculatedPollenPower);
        setPollenPitchPosition(calculatedPollenPitch);
        setNectarPower(calculatedNectarPower);
        setNectarPitchPosition(calculatedNectarPitch);
    }

    public Command periodic() {
        return infinite(() -> {
            if (isRunning) {
                if (pollenMotor != null) pollenMotor.setPower(pollenPower);
                if (nectarMotor != null) nectarMotor.setPower(nectarPower);
            } else {
                if (pollenMotor != null) pollenMotor.setPower(0.0);
                if (nectarMotor != null) nectarMotor.setPower(0.0);
            }

            if (pitchServoPollen != null) pitchServoPollen.setPosition(pollenPitchPosition);
            if (pitchServoNectar != null) pitchServoNectar.setPosition(nectarPitchPosition);
        });
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        telemetry.addData("Shooter Status", isRunning ? "ON" : "OFF");
        telemetry.addData("Pollen Shooter", String.format(java.util.Locale.US, "Pwr: %.2f | Pitch: %.3f", pollenPower, pollenPitchPosition));
        telemetry.addData("Nectar Shooter", String.format(java.util.Locale.US, "Pwr: %.2f | Pitch: %.3f", nectarPower, nectarPitchPosition));

        if (level == TelemetryLevel.DEBUG) {
            telemetry.addData("Pollen Motor Present", pollenMotor != null);
            telemetry.addData("Nectar Motor Present", nectarMotor != null);
            telemetry.addData("Pollen Servo Present", pitchServoPollen != null);
            telemetry.addData("Nectar Servo Present", pitchServoNectar != null);
        }
    }
}
