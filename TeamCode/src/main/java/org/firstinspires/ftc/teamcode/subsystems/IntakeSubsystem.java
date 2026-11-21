package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Config
@Configurable
public class IntakeSubsystem {

    public enum TelemetryLevel {
        NONE,
        MATCH,
        DEBUG
    }

    public static double fastPower = 1.0;
    public static double slowPower = 0.5;
    public static double reversePower = -1.0;
    public static long shortReverseTimeMs = 150;
    public static boolean reverseTransferMotor = false;

    public static double BAR_ZERO_POSITION = 0.1735;    // 847us
    public static double BAR_COLLECT_POSITION = 0.4725; // 1445us

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private Servo intakeBarServo;
    private Telemetry telemetry;

    private Mode mode = Mode.OFF;
    private boolean slowMode = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeBarServo = hardwareMap.get(Servo.class, "intakeBar");
        raiseBar();
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        transferMotor.setPower(reverseTransferMotor ? -power : power);
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
        });
    }

    public void raiseBar() {
        intakeBarServo.setPosition(BAR_ZERO_POSITION);
    }

    public void lowerBar() {
        intakeBarServo.setPosition(BAR_COLLECT_POSITION);
    }

    public void setBarPosition(double position) {
        intakeBarServo.setPosition(position);
    }

    public double getBarPosition() {
        return intakeBarServo.getPosition();
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        telemetry.addData("Intake Mode", mode + (slowMode ? " (SLOW)" : ""));
        telemetry.addData("Intake Bar Pos", intakeBarServo.getPosition());
    }

    public enum Mode {
        ON,
        OFF,
        REVERSE
    }
}
