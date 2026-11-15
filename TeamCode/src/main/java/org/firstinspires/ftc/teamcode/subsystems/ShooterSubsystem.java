package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.json.JSONObject;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Scanner;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
@Configurable
public class ShooterSubsystem {

    public enum TelemetryLevel {
        NONE,
        MATCH,
        DEBUG
    }

    public static String coefficientsSource = "DEFAULTS (Fișier negăsit)";

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

        loadCoefficientsFromJson();

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

    public static void loadCoefficientsFromJson() {
        try {
            File mainFile = new File(AppUtil.ROOT_FOLDER, "FIRST/shooter_coefficients.json");
            File bakFile = new File(AppUtil.ROOT_FOLDER, "FIRST/shooter_coefficients.bak");

            boolean isRecovered = false;

            // Auto-recuperare din .bak dacă .json lipsește
            if (!mainFile.exists() && bakFile.exists()) {
                boolean restored = bakFile.renameTo(mainFile);
                if (restored) {
                    isRecovered = true;
                }
            }

            if (!mainFile.exists()) {
                coefficientsSource = "DEFAULTS (Fișier JSON negăsit)";
                return;
            }

            Scanner scanner = new Scanner(new FileReader(mainFile));
            StringBuilder builder = new StringBuilder();
            while (scanner.hasNextLine()) {
                builder.append(scanner.nextLine());
            }
            scanner.close();

            JSONObject json = new JSONObject(builder.toString());

            boolean hasAll12Keys = json.has("pollen_power_a") && json.has("pollen_power_b") && json.has("pollen_power_c") &&
                                   json.has("pollen_pitch_a") && json.has("pollen_pitch_b") && json.has("pollen_pitch_c") &&
                                   json.has("nectar_power_a") && json.has("nectar_power_b") && json.has("nectar_power_c") &&
                                   json.has("nectar_pitch_a") && json.has("nectar_pitch_b") && json.has("nectar_pitch_c");

            POLLEN_POWER_A = json.optDouble("pollen_power_a", POLLEN_POWER_A);
            POLLEN_POWER_B = json.optDouble("pollen_power_b", POLLEN_POWER_B);
            POLLEN_POWER_C = json.optDouble("pollen_power_c", POLLEN_POWER_C);

            POLLEN_PITCH_A = json.optDouble("pollen_pitch_a", POLLEN_PITCH_A);
            POLLEN_PITCH_B = json.optDouble("pollen_pitch_b", POLLEN_PITCH_B);
            POLLEN_PITCH_C = json.optDouble("pollen_pitch_c", POLLEN_PITCH_C);

            NECTAR_POWER_A = json.optDouble("nectar_power_a", NECTAR_POWER_A);
            NECTAR_POWER_B = json.optDouble("nectar_power_b", NECTAR_POWER_B);
            NECTAR_POWER_C = json.optDouble("nectar_power_c", NECTAR_POWER_C);

            NECTAR_PITCH_A = json.optDouble("nectar_pitch_a", NECTAR_PITCH_A);
            NECTAR_PITCH_B = json.optDouble("nectar_pitch_b", NECTAR_PITCH_B);
            NECTAR_PITCH_C = json.optDouble("nectar_pitch_c", NECTAR_PITCH_C);

            boolean validFinite = Double.isFinite(POLLEN_POWER_A) && Double.isFinite(POLLEN_PITCH_A) &&
                                 Double.isFinite(NECTAR_POWER_A) && Double.isFinite(NECTAR_PITCH_A);

            if (hasAll12Keys && validFinite) {
                coefficientsSource = isRecovered
                        ? "JSON_RECUPERAT_DIN_BACKUP (shooter_coefficients.bak)"
                        : "JSON (Încărcat complet 12 chei din shooter_coefficients.json)";
            } else {
                coefficientsSource = "JSON_PARȚIAL (Lipsesc chei din fișier sau valori nefinite)";
            }
        } catch (Exception e) {
            coefficientsSource = "EROARE_CITIRE_JSON (Se folosesc valorile implicite)";
        }
    }

    public static boolean saveCoefficientsToJson(double[] pollenPower, double[] pollenPitch, double[] nectarPower, double[] nectarPitch) {
        try {
            boolean updatePollen = (pollenPower != null && pollenPower.length == 3 && pollenPitch != null && pollenPitch.length == 3);
            boolean updateNectar = (nectarPower != null && nectarPower.length == 3 && nectarPitch != null && nectarPitch.length == 3);

            if (!updatePollen && !updateNectar) {
                return false;
            }

            double newPollenPowerA = updatePollen ? pollenPower[0] : POLLEN_POWER_A;
            double newPollenPowerB = updatePollen ? pollenPower[1] : POLLEN_POWER_B;
            double newPollenPowerC = updatePollen ? pollenPower[2] : POLLEN_POWER_C;

            double newPollenPitchA = updatePollen ? pollenPitch[0] : POLLEN_PITCH_A;
            double newPollenPitchB = updatePollen ? pollenPitch[1] : POLLEN_PITCH_B;
            double newPollenPitchC = updatePollen ? pollenPitch[2] : POLLEN_PITCH_C;

            double newNectarPowerA = updateNectar ? nectarPower[0] : NECTAR_POWER_A;
            double newNectarPowerB = updateNectar ? nectarPower[1] : NECTAR_POWER_B;
            double newNectarPowerC = updateNectar ? nectarPower[2] : NECTAR_POWER_C;

            double newNectarPitchA = updateNectar ? nectarPitch[0] : NECTAR_PITCH_A;
            double newNectarPitchB = updateNectar ? nectarPitch[1] : NECTAR_PITCH_B;
            double newNectarPitchC = updateNectar ? nectarPitch[2] : NECTAR_PITCH_C;

            JSONObject json = new JSONObject();
            json.put("pollen_power_a", newPollenPowerA);
            json.put("pollen_power_b", newPollenPowerB);
            json.put("pollen_power_c", newPollenPowerC);

            json.put("pollen_pitch_a", newPollenPitchA);
            json.put("pollen_pitch_b", newPollenPitchB);
            json.put("pollen_pitch_c", newPollenPitchC);

            json.put("nectar_power_a", newNectarPowerA);
            json.put("nectar_power_b", newNectarPowerB);
            json.put("nectar_power_c", newNectarPowerC);

            json.put("nectar_pitch_a", newNectarPitchA);
            json.put("nectar_pitch_b", newNectarPitchB);
            json.put("nectar_pitch_c", newNectarPitchC);

            File firstDir = new File(AppUtil.ROOT_FOLDER, "FIRST");
            if (!firstDir.exists()) {
                firstDir.mkdirs();
            }

            File tmpFile = new File(firstDir, "shooter_coefficients.tmp");
            File finalFile = new File(firstDir, "shooter_coefficients.json");
            File bakFile = new File(firstDir, "shooter_coefficients.bak");

            PrintWriter writer = new PrintWriter(new FileWriter(tmpFile, false));
            writer.println(json.toString(2));
            writer.flush();
            boolean error = writer.checkError();
            writer.close();

            if (error) {
                if (tmpFile.exists()) tmpFile.delete();
                return false;
            }

            // Backup al fișierului vechi înainte de înlocuire
            if (finalFile.exists()) {
                if (bakFile.exists()) bakFile.delete();
                finalFile.renameTo(bakFile);
            }

            // Înlocuire sigură de fișiere (tmp -> final)
            boolean renamed = tmpFile.renameTo(finalFile);
            if (!renamed) {
                if (finalFile.exists()) {
                    finalFile.delete();
                }
                renamed = tmpFile.renameTo(finalFile);
            }

            if (renamed) {
                if (bakFile.exists()) bakFile.delete();

                if (updatePollen) {
                    POLLEN_POWER_A = newPollenPowerA;
                    POLLEN_POWER_B = newPollenPowerB;
                    POLLEN_POWER_C = newPollenPowerC;
                    POLLEN_PITCH_A = newPollenPitchA;
                    POLLEN_PITCH_B = newPollenPitchB;
                    POLLEN_PITCH_C = newPollenPitchC;
                }
                if (updateNectar) {
                    NECTAR_POWER_A = newNectarPowerA;
                    NECTAR_POWER_B = newNectarPowerB;
                    NECTAR_POWER_C = newNectarPowerC;
                    NECTAR_PITCH_A = newNectarPitchA;
                    NECTAR_PITCH_B = newNectarPitchB;
                    NECTAR_PITCH_C = newNectarPitchC;
                }
                coefficientsSource = "JSON (Salvat & Încărcat cu succes)";
                return true;
            } else {
                // Restaurare backup în caz de eșec
                if (bakFile.exists()) {
                    bakFile.renameTo(finalFile);
                }
                return false;
            }
        } catch (Exception ignored) {
            return false;
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
        telemetry.addData("Sursă Coeficienți", coefficientsSource);
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
