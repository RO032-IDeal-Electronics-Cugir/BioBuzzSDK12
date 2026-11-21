package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfig;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.PolynomialRegression;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

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
    private String lastRegressionSummary = "Niciun calcul efectuat (Apasă R1 + Triunghi pentru calcul automat)";

    private final List<Double> pollenDist = new ArrayList<>();
    private final List<Double> pollenPwr = new ArrayList<>();
    private final List<Double> pollenPitch = new ArrayList<>();

    private final List<Double> nectarDist = new ArrayList<>();
    private final List<Double> nectarPwr = new ArrayList<>();
    private final List<Double> nectarPitch = new ArrayList<>();

    private File pollenCsvFile;
    private File nectarCsvFile;

    @Override
    public void init() {
        super.init();

        Alliance.isAllianceSelected = true;

        File firstDir = new File(AppUtil.ROOT_FOLDER, "FIRST");
        if (!firstDir.exists()) {
            firstDir.mkdirs();
        }

        pollenCsvFile = new File(firstDir, "pollen_tuning.csv");
        nectarCsvFile = new File(firstDir, "nectar_tuning.csv");

        loadExistingCsvData();

        // Retine pozitia din autonomie daca exista; altfel aplica pozitia implicita conform alianței curente
        robot.drivetrain.usePreviousStartingPose();
        Pose currentPose = robot.drivetrain.getPose();
        if (Math.abs(currentPose.x()) < 1e-3 && Math.abs(currentPose.y()) < 1e-3 && Math.abs(currentPose.heading()) < 1e-3) {
            double startX = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_X : TurretConfig.RED_START_X;
            double startY = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_Y : TurretConfig.RED_START_Y;
            double startHeadingDeg = (robot.getAlliance() == Alliance.BLUE) ? TurretConfig.BLUE_START_HEADING_DEG : TurretConfig.RED_START_HEADING_DEG;

            robot.drivetrain.setStartingPose(new Pose(
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

        // Posibilitate resetare unghi turela la 0 grade in faza de INIT (Edge triggered)
        if (gamepad2.rightStickButtonWasPressed()) {
            boolean resetOk = robot.turret.resetEncoder();
            if (resetOk) {
                telemetry.addData("Turelă Status", "🟢 ORIGINE 0° RESETATĂ CU SUCCES!");
            } else {
                telemetry.addData("Turelă Status", "❌ EROARE: Hardware encoder turelă negăsit!");
            }
        }

        telemetry.addData("Tuning Mode", "OpMode pregătit pentru calibrare shooter.");
        telemetry.addData("Alianță curentă", robot.getAlliance());
        telemetry.addData("Puncte Pollen pe Disk", pollenPointsCount);
        telemetry.addData("Puncte Nectar pe Disk", nectarPointsCount);
        telemetry.addData("📂 Sursă Coeficienți Curenți", ShooterSubsystem.coefficientsSource);
        telemetry.addLine("------------------------------------------------");
        telemetry.addLine("📍 ATENȚIE: Orientează turela DREPT ÎNAINTE (0°) înainte de PLAY!");
        telemetry.addLine("   (sau apasă Gamepad 2 R3 pentru Reset Zero Turelă la 0°)");
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
        double strafe = gamepad1.left_stick_x;
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

        // Gamepad 2 - Toggle Flywheels ON / OFF & Intake/Transfer (Square / Patrat conform TuningTarget)
        if (gamepad2.squareWasPressed()) {
            switch (tuningTarget) {
                case POLLEN_ONLY:
                    robot.shooter.togglePollen();
                    break;
                case NECTAR_ONLY:
                    robot.shooter.toggleNectar();
                    break;
                case BOTH:
                    robot.shooter.toggle();
                    break;
            }

            if (robot.shooter.isRunning()) {
                robot.intake.on();
            } else {
                robot.intake.stop();
            }
        }

        // Gamepad 2 - Control Turelă (Cross = AUTO_AIM, Circle = MANUAL, R3 = RESET ZERO)
        if (gamepad2.crossWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.AUTO_AIM);
        } else if (gamepad2.circleWasPressed()) {
            robot.turret.setMode(TurretSubsystem.Mode.MANUAL);
        }

        if (gamepad2.rightStickButtonWasPressed()) {
            robot.turret.resetEncoder();
        }

        // Gamepad 2 - R1 + TRIUNGHI: CALCUL AUTOMAT REGRESIE & SALVARE JSON
        if (gamepad2.right_bumper && gamepad2.triangleWasPressed()) {
            calculateAndSaveRegressions();
        } else if (gamepad2.touchpad && gamepad2.triangleWasPressed()) {
            // Gamepad 2 - Touchpad + Triangle: Resetare fișiere CSV
            clearCsvFiles();
        } else if (gamepad2.triangleWasPressed()) {
            // Gamepad 2 - Triangle: Salvare Punct Curent în Fișierele CSV și Memorie
            saveCurrentTuningPoint();
        }

        // Telemetrie Detaliată de Calibrare
        printTuningTelemetry();
    }

    private void saveCurrentTuningPoint() {
        double distanceInches = robot.turret.getCurrentDistanceInches();

        if (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH) {
            double pwr = robot.shooter.getPollenPower();
            double pitch = robot.shooter.getPollenPitchPosition();
            String row = String.format(Locale.US, "%.6f,%.6f,%.6f", distanceInches, pwr, pitch);

            if (writeCsvRow(pollenCsvFile, row)) {
                pollenDist.add(distanceInches);
                pollenPwr.add(pwr);
                pollenPitch.add(pitch);
                pollenPointsCount = pollenDist.size();
                lastPollenEntry = String.format(Locale.US, "%.2f,%.3f,%.3f", distanceInches, pwr, pitch);
            } else {
                lastPollenEntry = "❌ EROARE SCRIERE CSV!";
            }
        }

        if (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH) {
            double pwr = robot.shooter.getNectarPower();
            double pitch = robot.shooter.getNectarPitchPosition();
            String row = String.format(Locale.US, "%.6f,%.6f,%.6f", distanceInches, pwr, pitch);

            if (writeCsvRow(nectarCsvFile, row)) {
                nectarDist.add(distanceInches);
                nectarPwr.add(pwr);
                nectarPitch.add(pitch);
                nectarPointsCount = nectarDist.size();
                lastNectarEntry = String.format(Locale.US, "%.2f,%.3f,%.3f", distanceInches, pwr, pitch);
            } else {
                lastNectarEntry = "❌ EROARE SCRIERE CSV!";
            }
        }
    }

    private void calculateAndSaveRegressions() {
        PolynomialRegression.FitResult fitPollenPwr = null;
        PolynomialRegression.FitResult fitPollenPitch = null;
        PolynomialRegression.FitResult fitNectarPwr = null;
        PolynomialRegression.FitResult fitNectarPitch = null;

        boolean calcPollen = (tuningTarget == TuningTarget.POLLEN_ONLY || tuningTarget == TuningTarget.BOTH);
        boolean calcNectar = (tuningTarget == TuningTarget.NECTAR_ONLY || tuningTarget == TuningTarget.BOTH);

        if (calcPollen) {
            fitPollenPwr = PolynomialRegression.fitQuadratic(pollenDist, pollenPwr);
            fitPollenPitch = PolynomialRegression.fitQuadratic(pollenDist, pollenPitch);
        }

        if (calcNectar) {
            fitNectarPwr = PolynomialRegression.fitQuadratic(nectarDist, nectarPwr);
            fitNectarPitch = PolynomialRegression.fitQuadratic(nectarDist, nectarPitch);
        }

        boolean pollenValid = calcPollen && fitPollenPwr != null && fitPollenPitch != null && fitPollenPwr.isValid && fitPollenPitch.isValid;
        boolean nectarValid = calcNectar && fitNectarPwr != null && fitNectarPitch != null && fitNectarPwr.isValid && fitNectarPitch.isValid;

        double[] pollenPowerCoeffs = pollenValid ? new double[]{fitPollenPwr.a, fitPollenPwr.b, fitPollenPwr.c} : null;
        double[] pollenPitchCoeffs = pollenValid ? new double[]{fitPollenPitch.a, fitPollenPitch.b, fitPollenPitch.c} : null;

        double[] nectarPowerCoeffs = nectarValid ? new double[]{fitNectarPwr.a, fitNectarPwr.b, fitNectarPwr.c} : null;
        double[] nectarPitchCoeffs = nectarValid ? new double[]{fitNectarPitch.a, fitNectarPitch.b, fitNectarPitch.c} : null;

        boolean saved = ShooterSubsystem.saveCoefficientsToJson(pollenPowerCoeffs, pollenPitchCoeffs, nectarPowerCoeffs, nectarPitchCoeffs);

        StringBuilder sb = new StringBuilder();
        if (saved) {
            sb.append("✅ SALVAT ÎN JSON DISK (Scriere Atomică)!\n");
        } else if (!pollenValid && !nectarValid) {
            sb.append("⚠️ NICIO CALIBRARE NOUĂ (Date insuficiente sau sub 3 distanțe cu separare >= 8in)!\n");
        } else {
            sb.append("❌ EROARE LA SCRIERE DISK JSON!\n");
        }

        if (pollenValid) {
            sb.append(String.format(Locale.US, "🌸 Pollen Pwr: %.2e x² + %.4f x + %.3f (R²=%.3f)\n", fitPollenPwr.a, fitPollenPwr.b, fitPollenPwr.c, fitPollenPwr.rSquared));
            sb.append(String.format(Locale.US, "🌸 Pollen Pitch: %.2e x² + %.4f x + %.3f (R²=%.3f)\n", fitPollenPitch.a, fitPollenPitch.b, fitPollenPitch.c, fitPollenPitch.rSquared));
        } else if (calcPollen) {
            sb.append("⚠️ POLLEN: Măsoară în min. 3 distanțe separate de cel puțin 8 in!\n");
        }

        if (nectarValid) {
            sb.append(String.format(Locale.US, "🍯 Nectar Pwr: %.2e x² + %.4f x + %.3f (R²=%.3f)\n", fitNectarPwr.a, fitNectarPwr.b, fitNectarPwr.c, fitNectarPwr.rSquared));
            sb.append(String.format(Locale.US, "🍯 Nectar Pitch: %.2e x² + %.4f x + %.3f (R²=%.3f)", fitNectarPitch.a, fitNectarPitch.b, fitNectarPitch.c, fitNectarPitch.rSquared));
        } else if (calcNectar) {
            sb.append("⚠️ NECTAR: Măsoară în min. 3 distanțe separate de cel puțin 8 in!");
        }

        lastRegressionSummary = sb.toString();
    }

    private void loadExistingCsvData() {
        pollenDist.clear();
        pollenPwr.clear();
        pollenPitch.clear();
        nectarDist.clear();
        nectarPwr.clear();
        nectarPitch.clear();

        readCsvIntoLists(pollenCsvFile, pollenDist, pollenPwr, pollenPitch);
        readCsvIntoLists(nectarCsvFile, nectarDist, nectarPwr, nectarPitch);

        pollenPointsCount = pollenDist.size();
        nectarPointsCount = nectarDist.size();

        if (pollenPointsCount > 0) {
            int idx = pollenPointsCount - 1;
            lastPollenEntry = String.format(Locale.US, "%.2f,%.3f,%.3f", pollenDist.get(idx), pollenPwr.get(idx), pollenPitch.get(idx));
        }
        if (nectarPointsCount > 0) {
            int idx = nectarPointsCount - 1;
            lastNectarEntry = String.format(Locale.US, "%.2f,%.3f,%.3f", nectarDist.get(idx), nectarPwr.get(idx), nectarPitch.get(idx));
        }
    }

    private void readCsvIntoLists(File file, List<Double> dist, List<Double> pwr, List<Double> pitch) {
        if (file == null || !file.exists()) return;

        try (Scanner scanner = new Scanner(new FileReader(file))) {
            if (scanner.hasNextLine()) {
                scanner.nextLine(); // Ignorare antet CSV
            }
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty()) continue;
                String[] parts = line.split(",");
                if (parts.length >= 3) {
                    try {
                        double d = Double.parseDouble(parts[0].trim());
                        double pw = Double.parseDouble(parts[1].trim());
                        double pt = Double.parseDouble(parts[2].trim());

                        if (Double.isFinite(d) && Double.isFinite(pw) && Double.isFinite(pt)) {
                            dist.add(d);
                            pwr.add(pw);
                            pitch.add(pt);
                        }
                    } catch (NumberFormatException ignored) {
                    }
                }
            }
        } catch (Exception ignored) {
        }
    }

    private boolean writeCsvRow(File file, String row) {
        try {
            boolean isNew = !file.exists() || file.length() == 0;
            PrintWriter writer = new PrintWriter(new FileWriter(file, true));
            if (isNew) {
                writer.println("Distance_in,Power,PitchPosition");
            }
            writer.println(row);
            writer.flush();
            boolean error = writer.checkError();
            writer.close();
            return !error;
        } catch (Exception e) {
            return false;
        }
    }

    private void clearCsvFiles() {
        try {
            boolean delPollen = !pollenCsvFile.exists() || pollenCsvFile.delete();
            boolean delNectar = !nectarCsvFile.exists() || nectarCsvFile.delete();

            if (delPollen) {
                pollenDist.clear();
                pollenPwr.clear();
                pollenPitch.clear();
                pollenPointsCount = 0;
                lastPollenEntry = "RESETAT";
            }

            if (delNectar) {
                nectarDist.clear();
                nectarPwr.clear();
                nectarPitch.clear();
                nectarPointsCount = 0;
                lastNectarEntry = "RESETAT";
            }

            if (delPollen && delNectar) {
                lastRegressionSummary = "Punctele au fost resetate din CSV și memorie.";
            } else if (delPollen) {
                lastRegressionSummary = "⚠️ Pollen resetat, dar eroare la ștergere Nectar CSV!";
            } else if (delNectar) {
                lastRegressionSummary = "⚠️ Nectar resetat, dar eroare la ștergere Pollen CSV!";
            } else {
                lastRegressionSummary = "❌ EROARE: Nu s-au putut șterge fișierele CSV de pe disk!";
            }
        } catch (Exception e) {
            telemetry.addData("CSV Reset Error", e.getMessage());
        }
    }

    private void printTuningTelemetry() {
        telemetry.addLine("═════════════ SHOOTER TUNING TELEOP ═════════════");
        telemetry.addData("🎯 Țintă Activă Tuning", tuningTarget);
        telemetry.addData("⚡ Flywheel Pollen", robot.shooter.isPollenRunning() ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("⚡ Flywheel Nectar", robot.shooter.isNectarRunning() ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("📂 Sursă Coeficienți Curenți", ShooterSubsystem.coefficientsSource);
        telemetry.addData("📏 Distanță calculată", String.format(Locale.US, "%.2f in (%.2f m)",
                robot.turret.getCurrentDistanceInches(), robot.turret.getCurrentDistanceInches() * 0.0254));

        telemetry.addLine("------------------------------------------------");
        telemetry.addData("🌸 POLLEN Shooter", String.format(Locale.US, "Power: %.2f | Pitch Servo: %.3f",
                robot.shooter.getPollenPower(), robot.shooter.getPollenPitchPosition()));
        telemetry.addData("🍯 NECTAR Shooter", String.format(Locale.US, "Power: %.2f | Pitch Servo: %.3f",
                robot.shooter.getNectarPower(), robot.shooter.getNectarPitchPosition()));

        telemetry.addLine("═════════════ COEFICIENȚI HARDWARE PIDF (REV HUB) ═════════════");
        telemetry.addData("⚙️ Pollen Default REV PIDF", ShooterSubsystem.defaultPollenPidf);
        telemetry.addData("⚙️ Nectar Default REV PIDF", ShooterSubsystem.defaultNectarPidf);
        if (ShooterConfig.ENABLE_CUSTOM_PIDF) {
            telemetry.addData("🎛️ Pollen Active PIDF", String.format(Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f",
                    ShooterConfig.POLLEN_PIDF_P, ShooterConfig.POLLEN_PIDF_I, ShooterConfig.POLLEN_PIDF_D, ShooterConfig.POLLEN_PIDF_F));
            telemetry.addData("🎛️ Nectar Active PIDF", String.format(Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f",
                    ShooterConfig.NECTAR_PIDF_P, ShooterConfig.NECTAR_PIDF_I, ShooterConfig.NECTAR_PIDF_D, ShooterConfig.NECTAR_PIDF_F));
        }

        telemetry.addLine("------------------------------------------------");
        telemetry.addData("🔄 Mod Turelă", robot.turret.getMode());
        telemetry.addData("📐 Unghi Turelă", String.format(Locale.US, "%.1f°", robot.turret.getTurretAngleDeg()));

        if (robot.drivetrain.getFollower() != null) {
            Pose pose = robot.drivetrain.getPose();
            telemetry.addData("📍 Poziție Robot (Pedro)", String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f°",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        }

        telemetry.addLine("═════════════ FIȘIERE CSV & CALCUL REGRESIE ═════════════");
        telemetry.addData("📊 Puncte Pollen Salvate", String.format(Locale.US, "%d (Ultimul: %s)", pollenPointsCount, lastPollenEntry));
        telemetry.addData("📊 Puncte Nectar Salvate", String.format(Locale.US, "%d (Ultimul: %s)", nectarPointsCount, lastNectarEntry));
        telemetry.addLine("------------------------------------------------");
        telemetry.addLine("🧮 STATUT REGRESIE AUTOMATĂ:");
        telemetry.addLine(lastRegressionSummary);

        telemetry.addLine("═════════════ CONTROALE GAMEPAD 2 ═════════════");
        telemetry.addLine("• L1 / Share: Comută POLLEN / NECTAR / BOTH");
        telemetry.addLine("• DPad Sus/Jos: ±Power | DPad Stânga/Dreapta: ±Pitch Servo");
        telemetry.addLine("• Pătrat: Pornire / Oprire Flywheels + Intake & Transfer");
        telemetry.addLine("• Triunghi: SALVARE PUNCT ÎN CSV & MEMORIE");
        telemetry.addLine("• R1 + Triunghi: CALCUL AUTOMAT REGRESIE & SALVARE JSON");
        telemetry.addLine("• R3 (Stick Dreapta): RESETARE ZERO TURELĂ (0° DREPT ÎNAINTE)");
        telemetry.addLine("• Touchpad + Triunghi: RESETARE PUNCTE & CSV");

        telemetry.update();
    }
}
