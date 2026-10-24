package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PDController;

@Config
@TeleOp(name = "Vision Align TeleOp", group = "Competition")
public class VisionAlignTeleOp extends OpMode {
    private Robot robot;
    private PDController pdController;

    public static double P_GAIN = 0.03;
    public static double D_GAIN = 0.001;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        pdController = new PDController(P_GAIN, D_GAIN);

        if (robot.vision != null) {
            robot.vision.setPipeline(1);
        }

        telemetry.addData("Status", "Vision Align TeleOp Initialized (Pipeline 1)");
        telemetry.update();
    }

    @Override
    public void loop() {
        pdController.setKp(P_GAIN);
        pdController.setKd(D_GAIN);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn;

        double error = 0.0;
        boolean hasTarget = robot.vision != null && robot.vision.hasTarget();

        if (hasTarget) {
            error = -robot.vision.getTargetX();
        }

        if (gamepad1.left_bumper && hasTarget) {
            turn = pdController.calculate(error);
            telemetry.addData("Vision Align Status", "ACTIVE");
        } else {
            turn = gamepad1.right_stick_x;
            pdController.reset();
            telemetry.addData("Vision Align Status", "MANUAL");
        }

        robot.drivetrain.drive(forward, strafe, turn);

        if (gamepad1.a) {
            robot.intake.setPower(1.0);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1.0);
        } else {
            robot.intake.stop();
        }

        if (gamepad1.x) {
            robot.shooter.setPower(0.85);
        } else {
            robot.shooter.stop();
        }

        robot.update();

        // Telemetrie vizibilă atât pe Driver Station, cât și în FTC Dashboard
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Heading Error (deg)", error);
        if (hasTarget) {
            telemetry.addData("Target Y (ty)", robot.vision.getTargetY());
        }
        telemetry.update();
    }
}
