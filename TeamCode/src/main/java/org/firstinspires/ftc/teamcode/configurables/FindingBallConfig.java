package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class FindingBallConfig {
    public static double P_GAIN = 0.015;
    public static double D_GAIN = 0.00001;
    public static double F_GAIN = 0.08;
    public static double TOLERANCE_DEGREES = 3.0;
    public static double VISION_ALPHA = 0.15;
}
