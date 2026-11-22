package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Configurable
public class ShooterConfig {
    public static String COEFFICIENTS_SOURCE = "DEFAULTS (Fișier negăsit)";

    // Specificații Hardware Motor (Encoder Ticks per Revoluție)
    // Implicit: 28.0 ticks/rev pentru motoare 1:1 (ex. GoBILDA 6000 RPM / REV HD Hex bare motor)
    public static double MOTOR_TICKS_PER_REV = 28.0;

    // Direcție Hardware Motoare (Motoare montate în oglindă pe robot)
    public static DcMotorSimple.Direction POLLEN_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction NECTAR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Coeficienți Hardware PIDF pentru Control Loop pe REV Hub
    public static boolean ENABLE_CUSTOM_PIDF = true;

    // Coeficienți PIDF Pollen Motor (P, I, D, F)
    public static double POLLEN_PIDF_P = 50.0;
    public static double POLLEN_PIDF_I = 0.0;
    public static double POLLEN_PIDF_D = 0.0;
    public static double POLLEN_PIDF_F = 15.0;

    // Coeficienți PIDF Nectar Motor (P, I, D, F)
    public static double NECTAR_PIDF_P = 50.0;
    public static double NECTAR_PIDF_I = 0.0;
    public static double NECTAR_PIDF_D = 0.0;
    public static double NECTAR_PIDF_F = 15.0;

    // Coeficienți Regresie Polinomială pentru Shooter POLLEN (Distanță in inchi -> RPM)
    public static double POLLEN_RPM_A = 0.0;
    public static double POLLEN_RPM_B = 20.0;
    public static double POLLEN_RPM_C = 1500.0;

    public static double POLLEN_PITCH_A = 0.0;
    public static double POLLEN_PITCH_B = 0.002;
    public static double POLLEN_PITCH_C = 0.40;

    // Coeficienți Regresie Polinomială pentru Shooter NECTAR (Distanță in inchi -> RPM)
    public static double NECTAR_RPM_A = 0.0;
    public static double NECTAR_RPM_B = 20.0;
    public static double NECTAR_RPM_C = 1500.0;

    public static double NECTAR_PITCH_A = 0.0;
    public static double NECTAR_PITCH_B = 0.002;
    public static double NECTAR_PITCH_C = 0.40;

    public static double TARGET_RPM = 3000.0;

    // Valori implicite Shooter & Limite Mecanice (Configurabile în FTCDashboard / Panels)
    public static double DEFAULT_POLLEN_RPM = 3500.0;
    public static double DEFAULT_NECTAR_RPM = 3500.0;
    public static double DEFAULT_POLLEN_PITCH = 0.80;
    public static double DEFAULT_NECTAR_PITCH = 0.80;

    public static double MAX_RPM = 6000.0;
    public static double MIN_RPM = 0.0;

    public static double MAX_PITCH_POSITION = 0.80;
    public static double MIN_PITCH_POSITION = 0.0;
}

