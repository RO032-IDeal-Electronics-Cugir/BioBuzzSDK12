package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class ShooterConfig {
    public static String COEFFICIENTS_SOURCE = "DEFAULTS (Fișier negăsit)";

    // Coeficienți Hardware PIDF pentru Control Loop pe REV Hub
    public static boolean ENABLE_CUSTOM_PIDF = true;

    // Coeficienți PIDF Pollen Motor (P, I, D, F)
    public static double POLLEN_PIDF_P = 10.0;
    public static double POLLEN_PIDF_I = 3.0;
    public static double POLLEN_PIDF_D = 0.0;
    public static double POLLEN_PIDF_F = 12.0;

    // Coeficienți PIDF Nectar Motor (P, I, D, F)
    public static double NECTAR_PIDF_P = 10.0;
    public static double NECTAR_PIDF_I = 3.0;
    public static double NECTAR_PIDF_D = 0.0;
    public static double NECTAR_PIDF_F = 12.0;

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

    public static double TARGET_POWER = 0.85;

    // Valori implicite Shooter & Limite Mecanice (Configurabile în FTCDashboard / Panels)
    public static double DEFAULT_POLLEN_POWER = 0.25;
    public static double DEFAULT_NECTAR_POWER = 0.25;
    public static double DEFAULT_POLLEN_PITCH = 0.80;
    public static double DEFAULT_NECTAR_PITCH = 0.80;

    public static double MAX_PITCH_POSITION = 0.80;
    public static double MIN_PITCH_POSITION = 0.0;
}
