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
}
