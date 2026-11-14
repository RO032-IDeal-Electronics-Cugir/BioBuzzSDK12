package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {

    public enum RobotTelemetryMode {
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
        turret = new TurretSubsystem(hardwareMap, telemetry, drivetrain != null ? drivetrain.getFollower() : null);
        turret.setAlliance(Alliance.current);
    }

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = null;

        drivetrain = new DrivetrainSubsystem(hardwareMap, null);
        intake = new IntakeSubsystem(hardwareMap, null);
        shooter = new ShooterSubsystem(hardwareMap, null);
        vision = new VisionSubsystem(hardwareMap, null);
        turret = new TurretSubsystem(hardwareMap, null, drivetrain != null ? drivetrain.getFollower() : null);
        turret.setAlliance(Alliance.current);
    }

    public void setAlliance(Alliance alliance) {
        Alliance.current = alliance;
        if (turret != null) {
            turret.setAlliance(alliance);
        }
    }

    public Alliance getAlliance() {
        return Alliance.current;
    }

    public void update() {
        if (drivetrain != null) {
            drivetrain.update();
        }
        if (vision != null) {
            vision.update();
        }
        if (turret != null && shooter != null && turret.getMode() == TurretSubsystem.Mode.AUTO_AIM) {
            shooter.updatePowerForDistance(turret.getCurrentDistanceInches());
        }
    }

    public void updateTelemetry() {
        updateTelemetry(RobotTelemetryMode.MATCH);
    }

    public void updateTelemetry(RobotTelemetryMode mode) {
        if (telemetry == null || mode == RobotTelemetryMode.OFF) return;

        telemetry.addData("Alliance", getAlliance());

        switch (mode) {
            case MATCH:
                if (drivetrain != null) drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.MATCH);
                if (turret != null) turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.MATCH);
                if (vision != null) vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.MATCH);
                if (shooter != null) shooter.printTelemetry(telemetry, ShooterSubsystem.TelemetryLevel.MATCH);
                if (intake != null) intake.printTelemetry(telemetry, IntakeSubsystem.TelemetryLevel.MATCH);
                break;

            case DEBUG_TURRET:
                if (turret != null) turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.DEBUG);
                if (vision != null) vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.MATCH);
                break;

            case DEBUG_VISION:
                if (vision != null) vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.DEBUG);
                break;

            case DEBUG_DRIVETRAIN:
                if (drivetrain != null) drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.DEBUG);
                break;

            case DEBUG_ALL:
                if (drivetrain != null) drivetrain.printTelemetry(telemetry, DrivetrainSubsystem.TelemetryLevel.DEBUG);
                if (turret != null) turret.printTelemetry(telemetry, TurretSubsystem.TelemetryLevel.DEBUG);
                if (vision != null) vision.printTelemetry(telemetry, VisionSubsystem.TelemetryLevel.DEBUG);
                if (shooter != null) shooter.printTelemetry(telemetry, ShooterSubsystem.TelemetryLevel.DEBUG);
                if (intake != null) intake.printTelemetry(telemetry, IntakeSubsystem.TelemetryLevel.DEBUG);
                break;
        }

        telemetry.update();
    }
}
