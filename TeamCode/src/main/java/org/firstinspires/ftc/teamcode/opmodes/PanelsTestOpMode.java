package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Panels Test OpMode", group = "Testing")
public class PanelsTestOpMode extends OpMode {
    
    // Variabilă statică pe care pluginul Configurables din Panels o poate citi/edita
    public static double TEST_VALUE = 42.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Panels Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("TEST_VALUE (Panels Configurable)", TEST_VALUE);
        telemetry.update();
    }
}
