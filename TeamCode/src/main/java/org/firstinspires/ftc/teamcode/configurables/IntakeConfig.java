package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConfig {
    public static double FAST_POWER = 1.0;
    public static double SLOW_POWER = 0.5;
    public static double REVERSE_POWER = -1.0;
    public static long SHORT_REVERSE_TIME_MS = 150;
    public static boolean REVERSE_TRANSFER_MOTOR = false;

    public static double BAR_ZERO_POSITION = 0.1735;    // 847us
    public static double BAR_COLLECT_POSITION = 0.4725; // 1445us
}
