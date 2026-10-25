package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Config
@Configurable
public class IntakeSubsystem {
    public static double fastPower = 1.0;
    public static double slowPower = 0.5;
    public static double reversePower = -1.0;
    public static long shortReverseTimeMs = 150;

    private DcMotor intakeMotor;
    private Telemetry telemetry;

    private Mode mode = Mode.OFF;
    private boolean slowMode = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intakeMotor = null;
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public void on() {
        mode = Mode.ON;
    }

    public void stop() {
        mode = Mode.OFF;
    }

    public void reverse() {
        mode = Mode.REVERSE;
    }

    public void toggle() {
        if (mode == Mode.ON) {
            stop();
        } else {
            on();
        }
    }

    public void slowDown() {
        slowMode = true;
    }

    public void speedUp() {
        slowMode = false;
    }

    public Command shortReverse() {
        return sequential(
                instant(this::reverse),
                waitMs(shortReverseTimeMs),
                instant(this::on)
        );
    }

    public Command periodic() {
        return infinite(() -> {
            double power = 0.0;
            switch (mode) {
                case ON:
                    power = slowMode ? slowPower : fastPower;
                    break;
                case REVERSE:
                    power = reversePower;
                    break;
                case OFF:
                default:
                    power = 0.0;
                    break;
            }

            setPower(power);

            if (telemetry != null) {
                telemetry.addData("Intake Mode", mode);
                telemetry.addData("Intake SlowMode", slowMode);
            }
        });
    }

    public enum Mode {
        ON,
        OFF,
        REVERSE
    }
}
