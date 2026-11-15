package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Locale;

@TeleOp(name = "Shooter Tuning TeleOp", group = "Competition")
public class ShooterTuningTeleOp extends RobotOpMode {

    public enum TuningTarget {
        POLLEN_ONLY,
        NECTAR_ONLY,
        BOTH
    }

    private TuningTarget tuningTarget = TuningTarget.POLLEN_ONLY;

    private int pollenPointsCount = 0;
    private int nectarPointsCount = 0;
    private String lastPollenEntry = "Niciunul";
    private String lastNectarEntry = "Niciunul";

    private File firstDir;
    private File pollenCsvFile;
    private File nectarCsvFile;

    @Override
    public void init() {
        super.init();

        Alliance.isAllianceSelected = true;

        firstDir = new File(AppUtil.ROOT_FOLDER, "FIRST");
        if (!firstDir.exists()) {
            firstDir.mkdirs();
        }

        pollenCsvFile = new File(firstDir, "pollen_tuning.csv");
        nectarCsvFile = new File(firstDir, "nectar_tuning.csv");

        // Retine pozitia din autonomie daca exista
        robot.drivetrain.usePreviousStartingPose();
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

        telemetry.addData("Tuning Mode", "OpMode pregătit pentru calibrare shooter.");
        telemetry.addData("Alianță curentă", robot.getAlliance());
        telemetry.addData("Instrucțiuni", "Selectează alianța (L1=RED, R1=BLUE) și apasă PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.turret.setMode(TurretSubsystem.Mode.AUTO_AIM);
        robot.isAutoShooterAdjustmentEnabled = false;
    }

    @Override
    public void loop() {
        super.loop();

        // Gamepad 1 - Control Drivetrain pentru pozitionare libera pe teren
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.shareWasPressed()) {
            robot.drivetrain.toggleDriveMode();
        }

        robot.drivetrain.drive(forward, strafe, turn);

        // Gamepad 2 - Comutare Mod Tuning (L1 / Share)
        if (gamepad2.leftBumperWasPressed() || gamepad2.shareWasPressed()) {
            switch (tuningTarget) {
                case POLLEN_ONLY:
                    tuningTarget = TuningTarget.NECTAR_ONLY;
                    break;
                case NECTAR_ONLY:
                    tuningTarget = TuningTarget.BOTH;
                    break;
                case BOTH:
                    tuningTarget = TuningTarget.POLLEN_ONLY;
                    break;
            }
        }

        // Gamepad 2 - Ajustare Putere (DPad Up / Down) cu pas fin (0.01 sau 0.05)
        double powerStep = gamepad2.right_bumper ? 0.05 : 0.01;
        if (gamepad2.dpadUpWasPressed()) {
            if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustPollenPower(powerStep);
            }
            if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustNectarPower(powerStep);
            }
        }
        if (gamepad2.dpadDownWasPressed()) {
            if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustPollenPower(-powerStep);
            }
            if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustNectarPower(-powerStep);
            }
        }

        // Gamepad 2 - Ajustare Unghi Pitch (DPad Left / Right) cu pas fin (0.005 sau 0.02)
        double pitchStep = gamepad2.right_bumper ? 0.02 : 0.005;
        if (gamepad2.dpadRightWasPressed()) {
            if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustPollenPitchPosition(pitchStep);
            }
            if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustNectarPitchPosition(pitchStep);
            }
        }
        if (gamepad2.dpadLeftWasPressed()) {
            if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustPollenPitchPosition(-pitchStep);
            }
            if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
                robot.shooter.adjustNectarPitchPosition(-pitchStep);
            }
        }

        // Gamepad 2 - Toggle Flywheels ON / OFF (Square / Patrat)
        if (gamepad2.squareWasPressed()) {
            robot.shooter.toggle();
        }

        // Gamepad 2 - Control Turelă (Cross = AUTO_AIM, Circle = MANUAL)
        if (gamepad2.crossWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.AUTO_AIM);
        } else if (gamepad2.circleWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.MANUAL);
        }

        // Gamepad 2 - Reset Fișiere CSV (Holding Touchpad + Triangle)
        if (gamepad2.touchpad && gamepad2.triangleWasPressed()) {
            clearCsvFiles();
        } else if (gamepad2.triangleWasPressed()) {
            // Gamepad 2 - Salvare Punct de Tuning în Fișierele CSV (Triangle)
            saveCurrentTuningPoint();
        }

        // Actualizare calcule robot (distanță turelă etc.)
        robot.update();

        // Telemetrie Detaliată de Calibrare
        printTuningTelemetry();
    }

    private void saveCurrentTuningPoint() {
        double distanceInches = robot.turret.getCurrentDistanceInches();

        if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
            double pwr = robot.shooter.getPollenPower();
            double pitch = robot.shooter.getPollenPitchPosition();
            String row = String.format(Locale.US, "%.2f,%.3f,%.3f", distanceInches, pwr, pitch);
            writeCsvRow(pollenCsvFile, row);
            pollenPointsCount++;
            lastPollenEntry = row;
        }

        if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
            double pwr = robot.shooter.getNectarPower();
            double pitch = robot.shooter.getNectarPitchPosition();
            String row = String.format(Locale.US, "%.2f,%.3f,%.3f", distanceInches, pwr, pitch);
            writeCsvRow(nectarCsvFile, row);
            nectarPointsCount++;
            lastNectarEntry = row;
        }
    }

    private void writeCsvRow(File file, String row) {
        try {
            boolean isNew = !file.exists() || file.length() == 0;
            PrintWriter writer = new PrintWriter(new FileWriter(file, true));
            if (isNew) {
                writer.println("Distance_in,Power,PitchPosition");
            }
            writer.println(row);
            writer.flush();
            writer.close();
        } catch (Exception e) {
            telemetry.addData("CSV Write Error", e.getMessage());
        }
    }

    private void clearCsvFiles() {
        try {
            if (pollenCsvFile.exists()) pollenCsvFile.delete();
            if (nectarCsvFile.exists()) nectarCsvFile.delete();
            pollenPointsCount = 0;
            nectarPointsCount = 0;
            lastPollenEntry = "RESETAT";
            lastNectarEntry = "RESETAT";
        } catch (Exception e) {
            telemetry.addData("CSV Reset Error", e.getMessage());
        }
    }

    private void printTuningTelemetry() {
        telemetry.addLine("═════════════ SHOOTER TUNING TELEOP ═════════════");
        telemetry.addData("🎯 Țintă Activă Tuning", tuningTarget);
        telemetry.addData("⚡ Flywheels Status", robot.shooter.isRunning() ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("📏 Distanță calculată", String.format(Locale.US, "%.2f in (%.2f m)",
                robot.turret.getCurrentDistanceInches(), robot.turret.getCurrentDistanceInches() * 0.0254));

        telemetry.addLine("------------------------------------------------");
        telemetry.addData("🌸 POLLEN Shooter", String.format(Locale.US, "Power: %.2f | Pitch Servo: %.3f",
                robot.shooter.getPollenPower(), robot.shooter.getPollenPitchPosition()));
        telemetry.addData("🍯 NECTAR Shooter", String.format(Locale.US, "Power: %.2f | Pitch Servo: %.3f",
                robot.shooter.getNectarPower(), robot.shooter.getNectarPitchPosition()));

        telemetry.addLine("------------------------------------------------");
        telemetry.addData("🔄 Mod Turelă", robot.turret.getMode());
        telemetry.addData("📐 Unghi Turelă", String.format(Locale.US, "%.1f°", robot.turret.getTurretAngleDeg()));

        if (robot.drivetrain != null && robot.drivetrain.getFollower() != null) {
            Pose pose = robot.drivetrain.getPose();
            telemetry.addData("📍 Poziție Robot (Pedro)", String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f°",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        }

        telemetry.addLine("═════════════ FIȘIERE CSV & PUNCTE ═════════════");
        telemetry.addData("📊 Puncte Pollen Salvate", String.format(Locale.US, "%d (Ultimul: %s)", pollenPointsCount, lastPollenEntry));
        telemetry.addData("📊 Puncte Nectar Salvate", String.format(Locale.US, "%d (Ultimul: %s)", nectarPointsCount, lastNectarEntry));

        telemetry.addLine("═════════════ CONTROALE GAMEPAD 2 ═════════════");
        telemetry.addLine("• L1 / Share: Comută POLLEN / NECTAR / BOTH");
        telemetry.addLine("• DPad Sus/Jos: ±Power (Hold R1 = pas mai mare)");
        telemetry.addLine("• DPad Stânga/Dreapta: ±Pitch Servo (Hold R1 = pas mai mare)");
        telemetry.addLine("• Pătrat: Pornire / Oprire Flywheels");
        telemetry.addLine("• Triunghi: SALVARE PUNCT ÎN CSV");
        telemetry.addLine("• Touchpad + Triunghi: RESETARE FIȘIERE CSV");

        telemetry.update();
    }
}
