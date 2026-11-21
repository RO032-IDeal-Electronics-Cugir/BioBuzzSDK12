package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConfig {
    public static String COEFFICIENTS_SOURCE = "DEFAULTS (Fișier negăsit)";

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
    public static double DEFAULT_POLLEN_POWER = 0.85;
    public static double DEFAULT_NECTAR_POWER = 0.85;
    public static double DEFAULT_POLLEN_PITCH = 0.80;
    public static double DEFAULT_NECTAR_PITCH = 0.80;

    public static double MAX_PITCH_POSITION = 0.80;
    public static double MIN_PITCH_POSITION = 0.0;
}
