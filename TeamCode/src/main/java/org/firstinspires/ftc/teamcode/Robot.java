package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import static com.pedropathing.ivy.commands.Commands.infinite;

public class Robot {

    public enum RobotTelemetryMode {
        INIT,
        MATCH,
        DEBUG_ALL,
        DEBUG_TURRET,
        DEBUG_VISION,
        DEBUG_DRIVETRAIN,
        OFF
    }

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public DrivetrainSubsystem drivetrain;
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;
    public TurretSubsystem turret;

    public boolean isAutoShooterAdjustmentEnabled = true;

    public Robot(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new MultipleTelemetry(
                opMode.telemetry,
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap, telemetry, drivetrain.getFollower());
        turret.setDrivetrain(drivetrain);
        turret.setAlliance(Alliance.current);
    }

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = null;

        drivetrain = new DrivetrainSubsystem(hardwareMap, null);
        intake = new IntakeSubsystem(hardwareMap, null);
        shooter = new ShooterSubsystem(hardwareMap, null);
        vision = new VisionSubsystem(hardwareMap, null);
        turret = new TurretSubsystem(hardwareMap, null, drivetrain.getFollower());
        turret.setDrivetrain(drivetrain);
        turret.setAlliance(Alliance.current);
    }

    public void setAlliance(Alliance alliance) {
        Alliance.current = alliance;
        Alliance.isAllianceSelected = true;
        turret.setAlliance(alliance);
    }

    public Alliance getAlliance() {
        return Alliance.current;
    }

    public Command periodic() {
        return infinite(() -> {
            if (turret.getMode() == TurretSubsystem.Mode.AUTO_AIM && isAutoShooterAdjustmentEnabled) {
                shooter.updatePowerForDistance(turret.getCurrentDistanceInches());
            }
        });
    }

    public void updateTelemetry() {
        updateTelemetry(RobotTelemetryMode.MATCH);
    }

    public void updateTelemetry(RobotTelemetryMode mode) {
        if (telemetry == null || mode == RobotTelemetryMode.OFF) return;

        telemetry.addData("Alliance", getAlliance());

        switch (mode) {
            case INIT:
                if (Alliance.isAllianceSelected) {
                    telemetry.addData("Status", "✅ ALIANȚĂ SELECTATĂ (" + getAlliance() + ")");
                    if (drivetrain.getFollower() != null) {
                        com.pedropathing.math.Pose pose = drivetrain.getPose();
                        telemetry.addData("Start Pose", String.format(java.util.Locale.US, "X: %.1f in, Y: %.1f in, H: %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
                    }
                    telemetry.addData("Info", "Robot pregătit. Apasă PLAY!");
                } else {
                    telemetry.addData("Status", "⚠️ ALIANȚĂ NESELECTATĂ!");
                    telemetry.addData("Atenție", "Apasă Gamepad 2 L1 (RED) sau R1 (BLUE)!");
                }
                break;

            case MATCH:
                drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.MATCH);
                turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.MATCH);
                vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.MATCH);
                shooter.printTelemetry(telemetry, ShooterSubsystem.TelemetryLevel.MATCH);
                intake.printTelemetry(telemetry, IntakeSubsystem.TelemetryLevel.MATCH);
                break;

            case DEBUG_TURRET:
                turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.DEBUG);
                vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.MATCH);
                break;

            case DEBUG_VISION:
                vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.DEBUG);
                break;

            case DEBUG_DRIVETRAIN:
                drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.DEBUG);
                break;

            case DEBUG_ALL:
                drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.DEBUG);
                turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.DEBUG);
                vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.DEBUG);
                shooter.printTelemetry(telemetry, ShooterSubsystem.TelemetryLevel.DEBUG);
                intake.printTelemetry(telemetry, IntakeSubsystem.TelemetryLevel.DEBUG);
                break;
        }

        telemetry.update();
    }
}
