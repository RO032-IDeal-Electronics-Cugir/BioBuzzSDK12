package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Robot TeleOp", group = "Competition")
public class TeleOpOpMode extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Intake controls
        if (gamepad1.a) {
            robot.intake.setPower(1.0);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1.0);
        } else {
            robot.intake.stop();
        }

        // Shooter controls
        if (gamepad1.x) {
            robot.shooter.setPower(0.85);
        } else {
            robot.shooter.stop();
        }

        robot.update();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
