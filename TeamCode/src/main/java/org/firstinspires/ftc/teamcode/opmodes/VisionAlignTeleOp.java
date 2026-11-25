package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.VisionAlignCommand;
import org.firstinspires.ftc.teamcode.configurables.FindingBallConfig;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.PDController;

@TeleOp(name = "Vision Align TeleOp (Command-Based)", group = "Competition")
public class VisionAlignTeleOp extends RobotOpMode {

    private PDController pdController;
    private VisionAlignCommand visionAlignCommand;

    @Override
    public void init() {
        super.init();
        Alliance.isAllianceSelected = false;

        pdController = new PDController(
                FindingBallConfig.P_GAIN,
                FindingBallConfig.D_GAIN,
                FindingBallConfig.F_GAIN,
                FindingBallConfig.TOLERANCE_DEGREES
        );
        visionAlignCommand = new VisionAlignCommand(robot, pdController);

        robot.intake.raiseBar();

        // Retine pozitia din autonomie daca exista; altfel aplica pozitia implicita conform alianței curente
        robot.drivetrain.usePreviousStartingPose();
        com.pedropathing.math.Pose currentPose = robot.drivetrain.getPose();
        if (Math.abs(currentPose.x()) < 1e-3 && Math.abs(currentPose.y()) < 1e-3 && Math.abs(currentPose.heading()) < 1e-3) {
            double startX = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_X : TurretConfig.RED_START_X;
            double startY = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_Y : TurretConfig.RED_START_Y;
            double startHeadingDeg = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_HEADING_DEG : TurretConfig.RED_START_HEADING_DEG;

            robot.drivetrain.setStartingPose(new com.pedropathing.math.Pose(
                    startX,
                    startY,
                    Math.toRadians(startHeadingDeg)
            ));
        }
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if (gamepad2.leftBumperWasPressed()) {
            robot.setAlliance(Alliance.RED);
            if (robot.drivetrain.getFollower() != null) {
                robot.drivetrain.setStartingPose(new Pose(
                        TurretConfig.RED_START_X,
                        TurretConfig.RED_START_Y,
                        Math.toRadians(TurretConfig.RED_START_HEADING_DEG)
                ));
            }
        } else if (gamepad2.rightBumperWasPressed()) {
            robot.setAlliance(Alliance.BLUE);
            if (robot.drivetrain.getFollower() != null) {
                robot.drivetrain.setStartingPose(new Pose(
                        TurretConfig.BLUE_START_X,
                        TurretConfig.BLUE_START_Y,
                        Math.toRadians(TurretConfig.BLUE_START_HEADING_DEG)
                ));
            }
        }

        robot.updateTelemetry(Robot.RobotTelemetryMode.INIT);
    }

    @Override
    public void start() {
        super.start();
        robot.isAutoShooterAdjustmentEnabled = true;

        if (!Alliance.isAllianceSelected) {
            robot.turret.setMode(TurretSubsystem.Mode.LOCK_FORWARD);
        }
    }
    @Override
    public void loop() {
        super.loop();

        pdController.setKp(FindingBallConfig.P_GAIN);
        pdController.setKd(FindingBallConfig.D_GAIN);
        pdController.setKf(FindingBallConfig.F_GAIN);
        pdController.setTolerance(FindingBallConfig.TOLERANCE_DEGREES);

        // Gamepad 1 - Control Drivetrain
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.shareWasPressed()) {
            robot.drivetrain.toggleDriveMode();
        }

        robot.drivetrain.drive(forward, strafe, turn);

        // Gamepad 1 - Shooter Control (R1 / Right Bumper = Pornire / Oprire Shootere)
        if (gamepad1.rightBumperWasPressed()) {
            robot.shooter.toggle();
        }

        // Gamepad 1 - Intake Control
        if (gamepad1.leftBumperWasPressed()) {
            robot.intake.shortReverse().schedule();
        } else if (!gamepad1.left_bumper) {
            if (gamepad1.cross) {
                robot.intake.on();
            } else if (gamepad1.circle) {
                robot.intake.reverse();
            } else {
                robot.intake.stop();
            }
        }

        // Gamepad 1 - Bara la intake
        if (gamepad1.dpadUpWasPressed()) {
            robot.intake.raiseBar();
        } else if (gamepad1.dpadDownWasPressed()) {
            robot.intake.lowerBar();
        }

        // Gamepad 2 - Schimbare Alianță
        if (gamepad2.leftBumperWasPressed()) {
            robot.setAlliance(Alliance.RED);
        }
        if (gamepad2.rightBumperWasPressed()) {
            robot.setAlliance(Alliance.BLUE);
        }

        // Gamepad 2 - Control Moduri Turelă
        if (gamepad2.crossWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.AUTO_AIM);
        } else if (gamepad2.squareWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.LOCK_FORWARD);
        } else if (gamepad2.circleWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.MANUAL);
        }

        // Gamepad 2 - Ajustare Manuală Turelă sau Putere Shooter
        if (robot.turret.getMode() == TurretSubsystem.Mode.MANUAL) {
            if (gamepad2.dpad_left) {
                robot.turret.adjustManualAngle(-1.0);
            } else if (gamepad2.dpad_right) {
                robot.turret.adjustManualAngle(1.0);
            }
        } else {
            if (gamepad2.dpadUpWasPressed()) {
                robot.shooter.increaseRPM(100);
            }
            if (gamepad2.dpadDownWasPressed()) {
                robot.shooter.decreaseRPM(100);
            }
        }

        // Gamepad 2 - Reset Heading Field-Centric
        if (gamepad2.triangleWasPressed()) {
            robot.drivetrain.resetHeading();
        }

        // Telemetrie Live Target & Pitch Calculat din Funcție
        double distanceInches = robot.turret.getCurrentDistanceInches();
        telemetry.addData("📏 Distanță Turelă -> HIVE", String.format(java.util.Locale.US, "%.2f in (%.2f m)", distanceInches, distanceInches * 0.0254));
        telemetry.addData("🌸 Pollen Target RPM", String.format(java.util.Locale.US, "%.0f RPM (Măsurat: %.0f RPM)", robot.shooter.getPollenRPM(), robot.shooter.getPollenCurrentRPM()));
        telemetry.addData("🌸 Pollen Target Pitch", String.format(java.util.Locale.US, "%.3f", robot.shooter.getPollenPitchPosition()));
        telemetry.addData("🍯 Nectar Target RPM", String.format(java.util.Locale.US, "%.0f RPM (Măsurat: %.0f RPM)", robot.shooter.getNectarRPM(), robot.shooter.getNectarCurrentRPM()));
        telemetry.addData("🍯 Nectar Target Pitch", String.format(java.util.Locale.US, "%.3f", robot.shooter.getNectarPitchPosition()));

        robot.updateTelemetry(Robot.RobotTelemetryMode.MATCH);
    }
}
