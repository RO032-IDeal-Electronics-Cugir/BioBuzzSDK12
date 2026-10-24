package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Dashboard Test OpMode", group = "Testing")
public class DashboardTestOpMode extends OpMode {

    public static double TEST_VALUE = 42.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Dashboard Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("TEST_VALUE from Dashboard", TEST_VALUE);
        telemetry.update();
    }
}
