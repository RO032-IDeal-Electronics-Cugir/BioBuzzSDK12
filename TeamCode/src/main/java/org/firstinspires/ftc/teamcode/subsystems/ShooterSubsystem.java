package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfig;

import org.json.JSONObject;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Scanner;

import static com.pedropathing.ivy.commands.Commands.infinite;

public class ShooterSubsystem {

    public enum TelemetryLevel {
        NONE,
        MATCH,
        DEBUG
    }

    public static String coefficientsSource = "DEFAULTS (Fișier negăsit)";
    public static String defaultPollenPidf = "N/A";
    public static String defaultNectarPidf = "N/A";

    private DcMotorEx pollenMotor;
    private DcMotorEx nectarMotor;
    private Servo pitchServoPollen;
    private Servo pitchServoNectar;

    private double pollenPower = ShooterConfig.DEFAULT_POLLEN_POWER;
    private double nectarPower = ShooterConfig.DEFAULT_NECTAR_POWER;
    private double pollenPitchPosition = ShooterConfig.DEFAULT_POLLEN_PITCH;
    private double nectarPitchPosition = ShooterConfig.DEFAULT_NECTAR_PITCH;

    private Telemetry telemetry;
    private boolean isPollenRunning = false;
    private boolean isNectarRunning = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        loadCoefficientsFromJson();

        this.pollenPower = ShooterConfig.DEFAULT_POLLEN_POWER;
        this.nectarPower = ShooterConfig.DEFAULT_NECTAR_POWER;
        this.pollenPitchPosition = Math.max(ShooterConfig.MIN_PITCH_POSITION, Math.min(ShooterConfig.MAX_PITCH_POSITION, ShooterConfig.DEFAULT_POLLEN_PITCH));
        this.nectarPitchPosition = Math.max(ShooterConfig.MIN_PITCH_POSITION, Math.min(ShooterConfig.MAX_PITCH_POSITION, ShooterConfig.DEFAULT_NECTAR_PITCH));

        try {
            pollenMotor = hardwareMap.get(DcMotorEx.class, "pollenMotor");
            pollenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pollenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            pollenMotor = null;
        }

        try {
            nectarMotor = hardwareMap.get(DcMotorEx.class, "nectarMotor");
            nectarMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            nectarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            nectarMotor = null;
        }

        fetchDefaultPIDF();
        updatePIDFCoefficients();

        pitchServoPollen = hardwareMap.get(Servo.class, "pitchServoPollen");
        pitchServoNectar = hardwareMap.get(Servo.class, "pitchServoNectar");

        if (pitchServoPollen != null) {
            pitchServoPollen.setPosition(this.pollenPitchPosition);
        }
        if (pitchServoNectar != null) {
            pitchServoNectar.setPosition(this.nectarPitchPosition);
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
                ShooterConfig.COEFFICIENTS_SOURCE = "DEFAULTS (Fișier JSON negăsit)";
                coefficientsSource = ShooterConfig.COEFFICIENTS_SOURCE;
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

            ShooterConfig.POLLEN_POWER_A = json.optDouble("pollen_power_a", ShooterConfig.POLLEN_POWER_A);
            ShooterConfig.POLLEN_POWER_B = json.optDouble("pollen_power_b", ShooterConfig.POLLEN_POWER_B);
            ShooterConfig.POLLEN_POWER_C = json.optDouble("pollen_power_c", ShooterConfig.POLLEN_POWER_C);

            ShooterConfig.POLLEN_PITCH_A = json.optDouble("pollen_pitch_a", ShooterConfig.POLLEN_PITCH_A);
            ShooterConfig.POLLEN_PITCH_B = json.optDouble("pollen_pitch_b", ShooterConfig.POLLEN_PITCH_B);
            ShooterConfig.POLLEN_PITCH_C = json.optDouble("pollen_pitch_c", ShooterConfig.POLLEN_PITCH_C);

            ShooterConfig.NECTAR_POWER_A = json.optDouble("nectar_power_a", ShooterConfig.NECTAR_POWER_A);
            ShooterConfig.NECTAR_POWER_B = json.optDouble("nectar_power_b", ShooterConfig.NECTAR_POWER_B);
            ShooterConfig.NECTAR_POWER_C = json.optDouble("nectar_power_c", ShooterConfig.NECTAR_POWER_C);

            ShooterConfig.NECTAR_PITCH_A = json.optDouble("nectar_pitch_a", ShooterConfig.NECTAR_PITCH_A);
            ShooterConfig.NECTAR_PITCH_B = json.optDouble("nectar_pitch_b", ShooterConfig.NECTAR_PITCH_B);
            ShooterConfig.NECTAR_PITCH_C = json.optDouble("nectar_pitch_c", ShooterConfig.NECTAR_PITCH_C);

            boolean validFinite = Double.isFinite(ShooterConfig.POLLEN_POWER_A) && Double.isFinite(ShooterConfig.POLLEN_PITCH_A) &&
                                 Double.isFinite(ShooterConfig.NECTAR_POWER_A) && Double.isFinite(ShooterConfig.NECTAR_PITCH_A);

            if (hasAll12Keys && validFinite) {
                ShooterConfig.COEFFICIENTS_SOURCE = isRecovered
                        ? "JSON_RECUPERAT_DIN_BACKUP (shooter_coefficients.bak)"
                        : "JSON (Încărcat complet 12 chei din shooter_coefficients.json)";
            } else {
                ShooterConfig.COEFFICIENTS_SOURCE = "JSON_PARȚIAL (Lipsesc chei din fișier sau valori nefinite)";
            }
        } catch (Exception e) {
            ShooterConfig.COEFFICIENTS_SOURCE = "EROARE_CITIRE_JSON (Se folosesc valorile implicite)";
        }
        coefficientsSource = ShooterConfig.COEFFICIENTS_SOURCE;
    }

    public static boolean saveCoefficientsToJson(double[] pollenPower, double[] pollenPitch, double[] nectarPower, double[] nectarPitch) {
        try {
            boolean updatePollen = (pollenPower != null && pollenPower.length == 3 && pollenPitch != null && pollenPitch.length == 3);
            boolean updateNectar = (nectarPower != null && nectarPower.length == 3 && nectarPitch != null && nectarPitch.length == 3);

            if (!updatePollen && !updateNectar) {
                return false;
            }

            double newPollenPowerA = updatePollen ? pollenPower[0] : ShooterConfig.POLLEN_POWER_A;
            double newPollenPowerB = updatePollen ? pollenPower[1] : ShooterConfig.POLLEN_POWER_B;
            double newPollenPowerC = updatePollen ? pollenPower[2] : ShooterConfig.POLLEN_POWER_C;

            double newPollenPitchA = updatePollen ? pollenPitch[0] : ShooterConfig.POLLEN_PITCH_A;
            double newPollenPitchB = updatePollen ? pollenPitch[1] : ShooterConfig.POLLEN_PITCH_B;
            double newPollenPitchC = updatePollen ? pollenPitch[2] : ShooterConfig.POLLEN_PITCH_C;

            double newNectarPowerA = updateNectar ? nectarPower[0] : ShooterConfig.NECTAR_POWER_A;
            double newNectarPowerB = updateNectar ? nectarPower[1] : ShooterConfig.NECTAR_POWER_B;
            double newNectarPowerC = updateNectar ? nectarPower[2] : ShooterConfig.NECTAR_POWER_C;

            double newNectarPitchA = updateNectar ? nectarPitch[0] : ShooterConfig.NECTAR_PITCH_A;
            double newNectarPitchB = updateNectar ? nectarPitch[1] : ShooterConfig.NECTAR_PITCH_B;
            double newNectarPitchC = updateNectar ? nectarPitch[2] : ShooterConfig.NECTAR_PITCH_C;

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
                    ShooterConfig.POLLEN_POWER_A = newPollenPowerA;
                    ShooterConfig.POLLEN_POWER_B = newPollenPowerB;
                    ShooterConfig.POLLEN_POWER_C = newPollenPowerC;
                    ShooterConfig.POLLEN_PITCH_A = newPollenPitchA;
                    ShooterConfig.POLLEN_PITCH_B = newPollenPitchB;
                    ShooterConfig.POLLEN_PITCH_C = newPollenPitchC;
                }
                if (updateNectar) {
                    ShooterConfig.NECTAR_POWER_A = newNectarPowerA;
                    ShooterConfig.NECTAR_POWER_B = newNectarPowerB;
                    ShooterConfig.NECTAR_POWER_C = newNectarPowerC;
                    ShooterConfig.NECTAR_PITCH_A = newNectarPitchA;
                    ShooterConfig.NECTAR_PITCH_B = newNectarPitchB;
                    ShooterConfig.NECTAR_PITCH_C = newNectarPitchC;
                }
                ShooterConfig.COEFFICIENTS_SOURCE = "JSON (Salvat & Încărcat cu succes)";
                coefficientsSource = ShooterConfig.COEFFICIENTS_SOURCE;
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
        ShooterConfig.TARGET_POWER = this.pollenPower;
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
        this.pollenPitchPosition = Math.max(ShooterConfig.MIN_PITCH_POSITION, Math.min(ShooterConfig.MAX_PITCH_POSITION, position));
        if (pitchServoPollen != null) {
            pitchServoPollen.setPosition(this.pollenPitchPosition);
        }
    }

    public void setNectarPitchPosition(double position) {
        this.nectarPitchPosition = Math.max(ShooterConfig.MIN_PITCH_POSITION, Math.min(ShooterConfig.MAX_PITCH_POSITION, position));
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

    public void turnOnPollen() {
        isPollenRunning = true;
    }

    public void turnOffPollen() {
        isPollenRunning = false;
    }

    public void togglePollen() {
        isPollenRunning = !isPollenRunning;
    }

    public boolean isPollenRunning() {
        return isPollenRunning;
    }

    public void turnOnNectar() {
        isNectarRunning = true;
    }

    public void turnOffNectar() {
        isNectarRunning = false;
    }

    public void toggleNectar() {
        isNectarRunning = !isNectarRunning;
    }

    public boolean isNectarRunning() {
        return isNectarRunning;
    }

    public void turnOn() {
        isPollenRunning = true;
        isNectarRunning = true;
    }

    public void turnOff() {
        isPollenRunning = false;
        isNectarRunning = false;
    }

    public void stop() {
        turnOff();
    }

    public void toggle() {
        if (isPollenRunning || isNectarRunning) {
            turnOff();
        } else {
            turnOn();
        }
    }

    public boolean isRunning() {
        return isPollenRunning || isNectarRunning;
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
        double calculatedPollenPower = ShooterConfig.POLLEN_POWER_A * d * d + ShooterConfig.POLLEN_POWER_B * d + ShooterConfig.POLLEN_POWER_C;
        double calculatedPollenPitch = ShooterConfig.POLLEN_PITCH_A * d * d + ShooterConfig.POLLEN_PITCH_B * d + ShooterConfig.POLLEN_PITCH_C;

        // Calcul independent pentru Nectar
        double calculatedNectarPower = ShooterConfig.NECTAR_POWER_A * d * d + ShooterConfig.NECTAR_POWER_B * d + ShooterConfig.NECTAR_POWER_C;
        double calculatedNectarPitch = ShooterConfig.NECTAR_PITCH_A * d * d + ShooterConfig.NECTAR_PITCH_B * d + ShooterConfig.NECTAR_PITCH_C;

        setPollenPower(calculatedPollenPower);
        setPollenPitchPosition(calculatedPollenPitch);
        setNectarPower(calculatedNectarPower);
        setNectarPitchPosition(calculatedNectarPitch);
    }

    public void fetchDefaultPIDF() {
        try {
            if (pollenMotor != null) {
                PIDFCoefficients pidf = pollenMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                defaultPollenPidf = String.format(java.util.Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f", pidf.p, pidf.i, pidf.d, pidf.f);
            }
        } catch (Exception e) {
            defaultPollenPidf = "Eroare citire";
        }

        try {
            if (nectarMotor != null) {
                PIDFCoefficients pidf = nectarMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                defaultNectarPidf = String.format(java.util.Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f", pidf.p, pidf.i, pidf.d, pidf.f);
            }
        } catch (Exception e) {
            defaultNectarPidf = "Eroare citire";
        }
    }

    public void updatePIDFCoefficients() {
        if (!ShooterConfig.ENABLE_CUSTOM_PIDF) return;

        if (pollenMotor != null) {
            pollenMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    ShooterConfig.POLLEN_PIDF_P,
                    ShooterConfig.POLLEN_PIDF_I,
                    ShooterConfig.POLLEN_PIDF_D,
                    ShooterConfig.POLLEN_PIDF_F
            ));
        }

        if (nectarMotor != null) {
            nectarMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    ShooterConfig.NECTAR_PIDF_P,
                    ShooterConfig.NECTAR_PIDF_I,
                    ShooterConfig.NECTAR_PIDF_D,
                    ShooterConfig.NECTAR_PIDF_F
            ));
        }
    }

    public Command periodic() {
        return infinite(() -> {
            updatePIDFCoefficients();

            pollenMotor.setPower(isPollenRunning ? pollenPower : 0.0);
            nectarMotor.setPower(isNectarRunning ? nectarPower : 0.0);

            pitchServoPollen.setPosition(pollenPitchPosition);
            pitchServoNectar.setPosition(nectarPitchPosition);
        });
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        telemetry.addData("Shooter Pollen", isPollenRunning ? "ON" : "OFF");
        telemetry.addData("Shooter Nectar", isNectarRunning ? "ON" : "OFF");
        telemetry.addData("Sursă Coeficienți", ShooterConfig.COEFFICIENTS_SOURCE);
        telemetry.addData("Pollen Shooter", String.format(java.util.Locale.US, "Pwr: %.2f | Pitch: %.3f", pollenPower, pollenPitchPosition));
        telemetry.addData("Nectar Shooter", String.format(java.util.Locale.US, "Pwr: %.2f | Pitch: %.3f", nectarPower, nectarPitchPosition));
        telemetry.addData("REV Default Pollen PIDF", defaultPollenPidf);
        telemetry.addData("REV Default Nectar PIDF", defaultNectarPidf);

        if (ShooterConfig.ENABLE_CUSTOM_PIDF) {
            telemetry.addData("Active Pollen PIDF", String.format(java.util.Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f",
                    ShooterConfig.POLLEN_PIDF_P, ShooterConfig.POLLEN_PIDF_I, ShooterConfig.POLLEN_PIDF_D, ShooterConfig.POLLEN_PIDF_F));
            telemetry.addData("Active Nectar PIDF", String.format(java.util.Locale.US, "P=%.2f, I=%.2f, D=%.2f, F=%.2f",
                    ShooterConfig.NECTAR_PIDF_P, ShooterConfig.NECTAR_PIDF_I, ShooterConfig.NECTAR_PIDF_D, ShooterConfig.NECTAR_PIDF_F));
        }
    }
}
