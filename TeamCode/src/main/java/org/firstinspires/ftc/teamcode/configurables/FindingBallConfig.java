package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class FindingBallConfig {
    public static double P_GAIN = 0.015;
    public static double D_GAIN = 0.00000;
    public static double F_GAIN = 0.00;
    public static double TOLERANCE_DEGREES = 3.0;
    public static double VISION_ALPHA = 0.15;

    // Geometry parameters in cm and degrees for distance calculation
    public static double CAMERA_HEIGHT_CM = 12.0;
    public static double BALL_CENTER_HEIGHT_CM = 4.0;
    public static double CAMERA_PITCH_DEGREES = 15.5;
}
