package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class TurretConfig {
    // Coordonatele coșurilor de țintire (în inchi) - Originea la (0,0) stânga-jos
    public static double RED_GOAL_BOTTOM_X = 58.0;
    public static double RED_GOAL_BOTTOM_Y = 58.0;
    public static double RED_GOAL_TOP_X = 58.0;
    public static double RED_GOAL_TOP_Y = 84.0;

    public static double BLUE_GOAL_BOTTOM_X = 84.0;
    public static double BLUE_GOAL_BOTTOM_Y = 58.0;
    public static double BLUE_GOAL_TOP_X = 84.0;
    public static double BLUE_GOAL_TOP_Y = 84.0;

    // Pozițiile de start standard (X inchi, Y inchi, Heading grade)
    public static double RED_START_X = 58.0;
    public static double RED_START_Y = 8.0;
    public static double RED_START_HEADING_DEG = 90.0;

    public static double BLUE_START_X = 132.0;
    public static double BLUE_START_Y = 36.0;
    public static double BLUE_START_HEADING_DEG = 180.0;

    // Parametri Mecanici & Senzori Turelă
    public static double TURRET_OFFSET_Y_CM = -10.0; // 10 cm în spatele centrului robotului
    public static double TICKS_PER_TURRET_REV = 24576.0; // 3 rotiri servo * 8192 tick-uri encoder REV
    public static double MAX_LIMIT_DEG = 270.0;
    public static double MIN_LIMIT_DEG = -270.0;

    // Sensuri Direcții & Inversări (Bifabil din Panels)
    public static boolean INVERT_ENCODER = false;
    public static boolean INVERT_SERVO = true;
    public static boolean ENABLED_IN_INIT = false;

    // Linia de mijloc și marja de histerezis pentru schimbarea coșului (sus/jos)
    public static double MIDLINE_Y = 71.0;
    public static double HYSTERESIS_MARGIN = 5.0;

    // Parametri Controller PD pentru Axon CRServo
    public static double P_GAIN = 0.03;
    public static double D_GAIN = 0.001;
    public static double TOLERANCE_DEG = 1.0;

    // Curba de putere Shooter în funcție de distanță (în inchi)
    public static double MIN_DISTANCE_INCHES = 24.0;
    public static double MAX_DISTANCE_INCHES = 120.0;
    public static double MIN_SHOOTER_POWER = 0.65;
    public static double MAX_SHOOTER_POWER = 0.95;
}
