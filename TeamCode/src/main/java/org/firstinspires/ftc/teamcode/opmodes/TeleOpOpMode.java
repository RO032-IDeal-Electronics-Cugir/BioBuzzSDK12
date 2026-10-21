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
        // Drivetrain control (Mecanum drive from Gamepad 1)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.drivetrain.drive(forward, strafe, turn);

        // Intake controls (A = Forward, B = Reverse)
        if (gamepad1.a) {
            robot.intake.setPower(1.0);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1.0);
        } else {
            robot.intake.stop();
        }

        // Shooter controls (X = Shoot)
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
