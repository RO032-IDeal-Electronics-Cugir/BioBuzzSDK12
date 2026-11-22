package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private double pollenRPM = ShooterConfig.DEFAULT_POLLEN_RPM;
    private double nectarRPM = ShooterConfig.DEFAULT_NECTAR_RPM;
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

        this.pollenRPM = ShooterConfig.DEFAULT_POLLEN_RPM;
        this.nectarRPM = ShooterConfig.DEFAULT_NECTAR_RPM;
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

    public static double rpmToTicksPerSec(double rpm) {
        return (rpm * ShooterConfig.MOTOR_TICKS_PER_REV) / 60.0;
    }

    public static double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec * 60.0) / ShooterConfig.MOTOR_TICKS_PER_REV;
    }

    public double getPollenCurrentRPM() {
        if (pollenMotor == null) return 0.0;
        try {
            return ticksPerSecToRpm(pollenMotor.getVelocity());
        } catch (Exception e) {
            return 0.0;
        }
    }

    public double getNectarCurrentRPM() {
        if (nectarMotor == null) return 0.0;
        try {
            return ticksPerSecToRpm(nectarMotor.getVelocity());
        } catch (Exception e) {
            return 0.0;
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

            boolean hasRpmKeys = json.has("pollen_rpm_a") || json.has("pollen_power_a");

            ShooterConfig.POLLEN_RPM_A = json.optDouble("pollen_rpm_a", json.optDouble("pollen_power_a", ShooterConfig.POLLEN_RPM_A));
            ShooterConfig.POLLEN_RPM_B = json.optDouble("pollen_rpm_b", json.optDouble("pollen_power_b", ShooterConfig.POLLEN_RPM_B));
            ShooterConfig.POLLEN_RPM_C = json.optDouble("pollen_rpm_c", json.optDouble("pollen_power_c", ShooterConfig.POLLEN_RPM_C));

            ShooterConfig.POLLEN_PITCH_A = json.optDouble("pollen_pitch_a", ShooterConfig.POLLEN_PITCH_A);
            ShooterConfig.POLLEN_PITCH_B = json.optDouble("pollen_pitch_b", ShooterConfig.POLLEN_PITCH_B);
            ShooterConfig.POLLEN_PITCH_C = json.optDouble("pollen_pitch_c", ShooterConfig.POLLEN_PITCH_C);

            ShooterConfig.NECTAR_RPM_A = json.optDouble("nectar_rpm_a", json.optDouble("nectar_power_a", ShooterConfig.NECTAR_RPM_A));
            ShooterConfig.NECTAR_RPM_B = json.optDouble("nectar_rpm_b", json.optDouble("nectar_power_b", ShooterConfig.NECTAR_RPM_B));
            ShooterConfig.NECTAR_RPM_C = json.optDouble("nectar_rpm_c", json.optDouble("nectar_power_c", ShooterConfig.NECTAR_RPM_C));

            ShooterConfig.NECTAR_PITCH_A = json.optDouble("nectar_pitch_a", ShooterConfig.NECTAR_PITCH_A);
            ShooterConfig.NECTAR_PITCH_B = json.optDouble("nectar_pitch_b", ShooterConfig.NECTAR_PITCH_B);
            ShooterConfig.NECTAR_PITCH_C = json.optDouble("nectar_pitch_c", ShooterConfig.NECTAR_PITCH_C);

            boolean validFinite = Double.isFinite(ShooterConfig.POLLEN_RPM_A) && Double.isFinite(ShooterConfig.POLLEN_PITCH_A) &&
                                 Double.isFinite(ShooterConfig.NECTAR_RPM_A) && Double.isFinite(ShooterConfig.NECTAR_PITCH_A);

            if (hasRpmKeys && validFinite) {
                ShooterConfig.COEFFICIENTS_SOURCE = isRecovered
                        ? "JSON_RECUPERAT_DIN_BACKUP (shooter_coefficients.bak)"
                        : "JSON (Încărcat complet din shooter_coefficients.json)";
            } else {
                ShooterConfig.COEFFICIENTS_SOURCE = "JSON_PARȚIAL (Lipsesc chei din fișier sau valori nefinite)";
            }
        } catch (Exception e) {
            ShooterConfig.COEFFICIENTS_SOURCE = "EROARE_CITIRE_JSON (Se folosesc valorile implicite)";
        }
        coefficientsSource = ShooterConfig.COEFFICIENTS_SOURCE;
    }

    public static boolean saveCoefficientsToJson(double[] pollenRpm, double[] pollenPitch, double[] nectarRpm, double[] nectarPitch) {
        try {
            boolean updatePollen = (pollenRpm != null && pollenRpm.length == 3 && pollenPitch != null && pollenPitch.length == 3);
            boolean updateNectar = (nectarRpm != null && nectarRpm.length == 3 && nectarPitch != null && nectarPitch.length == 3);

            if (!updatePollen && !updateNectar) {
                return false;
            }

            double newPollenRpmA = updatePollen ? pollenRpm[0] : ShooterConfig.POLLEN_RPM_A;
            double newPollenRpmB = updatePollen ? pollenRpm[1] : ShooterConfig.POLLEN_RPM_B;
            double newPollenRpmC = updatePollen ? pollenRpm[2] : ShooterConfig.POLLEN_RPM_C;

            double newPollenPitchA = updatePollen ? pollenPitch[0] : ShooterConfig.POLLEN_PITCH_A;
            double newPollenPitchB = updatePollen ? pollenPitch[1] : ShooterConfig.POLLEN_PITCH_B;
            double newPollenPitchC = updatePollen ? pollenPitch[2] : ShooterConfig.POLLEN_PITCH_C;

            double newNectarRpmA = updateNectar ? nectarRpm[0] : ShooterConfig.NECTAR_RPM_A;
            double newNectarRpmB = updateNectar ? nectarRpm[1] : ShooterConfig.NECTAR_RPM_B;
            double newNectarRpmC = updateNectar ? nectarRpm[2] : ShooterConfig.NECTAR_RPM_C;

            double newNectarPitchA = updateNectar ? nectarPitch[0] : ShooterConfig.NECTAR_PITCH_A;
            double newNectarPitchB = updateNectar ? nectarPitch[1] : ShooterConfig.NECTAR_PITCH_B;
            double newNectarPitchC = updateNectar ? nectarPitch[2] : ShooterConfig.NECTAR_PITCH_C;

            JSONObject json = new JSONObject();
            json.put("pollen_rpm_a", newPollenRpmA);
            json.put("pollen_rpm_b", newPollenRpmB);
            json.put("pollen_rpm_c", newPollenRpmC);

            json.put("pollen_pitch_a", newPollenPitchA);
            json.put("pollen_pitch_b", newPollenPitchB);
            json.put("pollen_pitch_c", newPollenPitchC);

            json.put("nectar_rpm_a", newNectarRpmA);
            json.put("nectar_rpm_b", newNectarRpmB);
            json.put("nectar_rpm_c", newNectarRpmC);

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

            if (finalFile.exists()) {
                if (bakFile.exists()) bakFile.delete();
                finalFile.renameTo(bakFile);
            }

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
                    ShooterConfig.POLLEN_RPM_A = newPollenRpmA;
                    ShooterConfig.POLLEN_RPM_B = newPollenRpmB;
                    ShooterConfig.POLLEN_RPM_C = newPollenRpmC;
                    ShooterConfig.POLLEN_PITCH_A = newPollenPitchA;
                    ShooterConfig.POLLEN_PITCH_B = newPollenPitchB;
                    ShooterConfig.POLLEN_PITCH_C = newPollenPitchC;
                }
                if (updateNectar) {
                    ShooterConfig.NECTAR_RPM_A = newNectarRpmA;
                    ShooterConfig.NECTAR_RPM_B = newNectarRpmB;
                    ShooterConfig.NECTAR_RPM_C = newNectarRpmC;
                    ShooterConfig.NECTAR_PITCH_A = newNectarPitchA;
                    ShooterConfig.NECTAR_PITCH_B = newNectarPitchB;
                    ShooterConfig.NECTAR_PITCH_C = newNectarPitchC;
                }
                ShooterConfig.COEFFICIENTS_SOURCE = "JSON (Salvat & Încărcat cu succes)";
                coefficientsSource = ShooterConfig.COEFFICIENTS_SOURCE;
                return true;
            } else {
                if (bakFile.exists()) {
                    bakFile.renameTo(finalFile);
                }
                return false;
            }
        } catch (Exception ignored) {
            return false;
        }
    }

    public void setRPM(double rpm) {
        setPollenRPM(rpm);
        setNectarRPM(rpm);
    }

    public void setPollenRPM(double rpm) {
        this.pollenRPM = Math.max(ShooterConfig.MIN_RPM, Math.min(ShooterConfig.MAX_RPM, rpm));
        ShooterConfig.TARGET_RPM = this.pollenRPM;
    }

    public void setNectarRPM(double rpm) {
        this.nectarRPM = Math.max(ShooterConfig.MIN_RPM, Math.min(ShooterConfig.MAX_RPM, rpm));
    }

    public double getPollenRPM() {
        return pollenRPM;
    }

    public double getNectarRPM() {
        return nectarRPM;
    }

    @Deprecated
    public void setPower(double power) {
        setRPM(power * ShooterConfig.MAX_RPM);
    }

    @Deprecated
    public void setPollenPower(double power) {
        setPollenRPM(power * ShooterConfig.MAX_RPM);
    }

    @Deprecated
    public void setNectarPower(double power) {
        setNectarRPM(power * ShooterConfig.MAX_RPM);
    }

    @Deprecated
    public double getPollenPower() {
        return pollenRPM / ShooterConfig.MAX_RPM;
    }

    @Deprecated
    public double getNectarPower() {
        return nectarRPM / ShooterConfig.MAX_RPM;
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

    public void adjustPollenRPM(double delta) {
        setPollenRPM(pollenRPM + delta);
    }

    public void adjustNectarRPM(double delta) {
        setNectarRPM(nectarRPM + delta);
    }

    @Deprecated
    public void adjustPollenPower(double delta) {
        adjustPollenRPM(delta * ShooterConfig.MAX_RPM);
    }

    @Deprecated
    public void adjustNectarPower(double delta) {
        adjustNectarRPM(delta * ShooterConfig.MAX_RPM);
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

    public void increaseRPM(double delta) {
        adjustPollenRPM(delta);
        adjustNectarRPM(delta);
    }

    public void decreaseRPM(double delta) {
        adjustPollenRPM(-delta);
        adjustNectarRPM(-delta);
    }

    @Deprecated
    public void increasePower(double delta) {
        increaseRPM(delta * ShooterConfig.MAX_RPM);
    }

    @Deprecated
    public void decreasePower(double delta) {
        decreaseRPM(delta * ShooterConfig.MAX_RPM);
    }

    public void updateSpeedForDistance(double distanceInches) {
        double d = Math.max(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MIN_DISTANCE_INCHES,
                   Math.min(org.firstinspires.ftc.teamcode.configurables.TurretConfig.MAX_DISTANCE_INCHES, distanceInches));

        double calculatedPollenRPM = ShooterConfig.POLLEN_RPM_A * d * d + ShooterConfig.POLLEN_RPM_B * d + ShooterConfig.POLLEN_RPM_C;
        double calculatedPollenPitch = ShooterConfig.POLLEN_PITCH_A * d * d + ShooterConfig.POLLEN_PITCH_B * d + ShooterConfig.POLLEN_PITCH_C;

        double calculatedNectarRPM = ShooterConfig.NECTAR_RPM_A * d * d + ShooterConfig.NECTAR_RPM_B * d + ShooterConfig.NECTAR_RPM_C;
        double calculatedNectarPitch = ShooterConfig.NECTAR_PITCH_A * d * d + ShooterConfig.NECTAR_PITCH_B * d + ShooterConfig.NECTAR_PITCH_C;

        setPollenRPM(calculatedPollenRPM);
        setPollenPitchPosition(calculatedPollenPitch);
        setNectarRPM(calculatedNectarRPM);
        setNectarPitchPosition(calculatedNectarPitch);
    }

    public void updatePowerForDistance(double distanceInches) {
        updateSpeedForDistance(distanceInches);
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

            if (pollenMotor != null) {
                double targetTicks = isPollenRunning ? rpmToTicksPerSec(pollenRPM) : 0.0;
                pollenMotor.setVelocity(targetTicks);
            }
            if (nectarMotor != null) {
                double targetTicks = isNectarRunning ? rpmToTicksPerSec(nectarRPM) : 0.0;
                nectarMotor.setVelocity(targetTicks);
            }

            if (pitchServoPollen != null) {
                pitchServoPollen.setPosition(pollenPitchPosition);
            }
            if (pitchServoNectar != null) {
                pitchServoNectar.setPosition(nectarPitchPosition);
            }
        });
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        // Telemetrie Numerica (Plotting în FTC Dashboard / Panels)
        telemetry.addData("Pollen Target RPM", isPollenRunning ? pollenRPM : 0.0);
        telemetry.addData("Pollen Current RPM", getPollenCurrentRPM());
        telemetry.addData("Nectar Target RPM", isNectarRunning ? nectarRPM : 0.0);
        telemetry.addData("Nectar Current RPM", getNectarCurrentRPM());

        telemetry.addData("Shooter Pollen", isPollenRunning ? "ON" : "OFF");
        telemetry.addData("Shooter Nectar", isNectarRunning ? "ON" : "OFF");
        telemetry.addData("Sursă Coeficienți", ShooterConfig.COEFFICIENTS_SOURCE);
        telemetry.addData("Pollen Shooter", String.format(java.util.Locale.US, "RPM: %.0f (Curent: %.0f) | Pitch: %.3f", pollenRPM, getPollenCurrentRPM(), pollenPitchPosition));
        telemetry.addData("Nectar Shooter", String.format(java.util.Locale.US, "RPM: %.0f (Curent: %.0f) | Pitch: %.3f", nectarRPM, getNectarCurrentRPM(), nectarPitchPosition));
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
