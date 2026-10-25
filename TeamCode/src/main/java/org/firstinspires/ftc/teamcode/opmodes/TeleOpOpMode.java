package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@TeleOp(name = "Robot TeleOp (Basic)", group = "Competition")
public class TeleOpOpMode extends RobotOpMode {

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Basic TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        super.loop();

        // Drivetrain control (Mecanum drive from Gamepad 1)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.drivetrain.drive(forward, strafe, turn);

        // Intake controls (Cross = Forward, Circle = Reverse)
        if (gamepad1.cross) {
            robot.intake.on();
        } else if (gamepad1.circle) {
            robot.intake.reverse();
        } else {
            robot.intake.stop();
        }

        // Shooter controls (Square = Toggle)
        if (gamepad1.square) {
            robot.shooter.turnOn();
        } else {
            robot.shooter.turnOff();
        }

        robot.updateTelemetry();
    }
}
