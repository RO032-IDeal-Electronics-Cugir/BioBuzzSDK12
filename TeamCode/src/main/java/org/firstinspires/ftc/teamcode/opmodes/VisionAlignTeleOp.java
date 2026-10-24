package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PDController;

@Configurable
@TeleOp(name = "Vision Align TeleOp", group = "Competition")
public class VisionAlignTeleOp extends OpMode {
    private Robot robot;
    private PDController pdController;

    public static double P_GAIN = 0.015;
    public static double D_GAIN = 0.00001;

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
        robot.update();

        pdController.setKp(P_GAIN);
        pdController.setKd(D_GAIN);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn;

        double error = 0.0;
        boolean hasTarget = robot.vision != null && robot.vision.hasTarget();

        if (hasTarget) {
            error = robot.vision.getTargetX();
        }

        //if (gamepad1.left_bumper && hasTarget) {
        if (hasTarget) {
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

        // Telemetrie vizibilă în Panels și Driver Station
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Heading Error (deg)", error);
        telemetry.addData("Current P_GAIN", P_GAIN);
        telemetry.addData("Current D_GAIN", D_GAIN);
        if (hasTarget) {
            telemetry.addData("Target Y (ty)", robot.vision.getTargetY());
        }
        telemetry.update();
    }
}
