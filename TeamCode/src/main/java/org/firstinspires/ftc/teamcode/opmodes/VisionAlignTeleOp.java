package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        // Folosim MultipleTelemetry pentru a trimite telemetria simultan
        // atât pe Driver Station, cât și în FTC Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap);
        pdController = new PDController(P_GAIN, D_GAIN);

        if (robot.vision != null) {
            robot.vision.setPipeline(1);
        }

        telemetry.addData("Status", "Vision Align TeleOp Initialized");
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

        if (gamepad1.left_bumper && hasTarget) {
            turn = -pdController.calculate(error);
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

        // Telemetria va apărea automat și pe DS și în Dashboard
        telemetry.addData("1. Has Target", hasTarget);
        telemetry.addData("2. Heading Error (tx)", error);
        telemetry.addData("3. Current P_GAIN", P_GAIN);
        telemetry.addData("4. Current D_GAIN", D_GAIN);
        telemetry.addData("5. Calculated Turn Power", turn);
        if (hasTarget) {
            telemetry.addData("6. Target Y (ty)", robot.vision.getTargetY());
        }
        telemetry.update();
    }
}
