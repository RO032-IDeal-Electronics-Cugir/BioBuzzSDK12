package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.VisionAlignCommand;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.PDController;

@Config
@Configurable
@TeleOp(name = "Vision Align TeleOp (Command-Based)", group = "Competition")
public class VisionAlignTeleOp extends RobotOpMode {

    public static double P_GAIN = 0.015;
    public static double D_GAIN = 0.00001;
    public static double F_GAIN = 0.08; // Feedforward minimum power (Panels / FTC Dashboard)
    public static double TOLERANCE_DEGREES = 3; // Angle tolerance deadband (Panels / FTC Dashboard)
    public static double VISION_ALPHA = 0.15; // EMA Low-Pass Filter smoothing factor (Panels / FTC Dashboard)

    private PDController pdController;
    private VisionAlignCommand visionAlignCommand;

    private boolean isAlignActive = false;

    // Direct edge detection flags (PlayStation controller mappings)
    private boolean prevG1Share = false;
    private boolean prevG1LB = false;
    private boolean prevG1Square = false;
    private boolean prevG2Triangle = false;
    private boolean prevG2LB = false;
    private boolean prevG2RB = false;
    private boolean prevG2DUp = false;
    private boolean prevG2DDown = false;

    @Override
    public void init() {
        super.init();

        pdController = new PDController(P_GAIN, D_GAIN, F_GAIN, TOLERANCE_DEGREES);
        visionAlignCommand = new VisionAlignCommand(robot, pdController);

        // Retine pozitia de la autonom daca exista
        robot.drivetrain.usePreviousStartingPose();

        telemetry.addData("Status", "Vision Align TeleOp Initialized (PS Controllers)");
        telemetry.update();
    }

    @Override
    public void loop() {
        super.loop();

        pdController.setKp(P_GAIN);
        pdController.setKd(D_GAIN);
        pdController.setKf(F_GAIN);
        pdController.setTolerance(TOLERANCE_DEGREES);

        // Gamepad 1 - Control Drivetrain
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Comutare Mode Conducere (Field-Centric vs Robot-Centric) cu Share (Back)
        if (gamepad1.share && !prevG1Share) {
            robot.drivetrain.toggleDriveMode();
        }
        prevG1Share = gamepad1.share;

        // Auto-Align cu Limelight cand se tine apasat Right Trigger (R2)
        boolean triggerPressed = gamepad1.right_trigger > 0.3;
        if (triggerPressed) {
            if (!isAlignActive) {
                isAlignActive = true;
                visionAlignCommand.start();
            }
            visionAlignCommand.execute(forward, strafe, TOLERANCE_DEGREES, VISION_ALPHA);

            boolean aligned = visionAlignCommand.isAligned(TOLERANCE_DEGREES, VISION_ALPHA);
            telemetry.addData("Vision Align", aligned ? "ALIGNED (In Tolerance)" : "ALIGNING (In Progress)");
        } else {
            if (isAlignActive) {
                isAlignActive = false;
                visionAlignCommand.stop();
            }
            robot.drivetrain.drive(forward, strafe, turn);
            telemetry.addData("Vision Align", "MANUAL (Goal Pipeline 0)");
        }

        // Gamepad 1 - Intake Control (Cross = In, Circle = Out, L1 = Short Reverse)
        if (gamepad1.left_bumper && !prevG1LB) {
            robot.intake.shortReverse().schedule();
        } else if (!gamepad1.left_bumper) {
            if (gamepad1.cross) {
                robot.intake.on();
            } else if (gamepad1.circle) {
                robot.intake.reverse();
            } else {
                robot.intake.stop();
            }
        }
        prevG1LB = gamepad1.left_bumper;

        // Gamepad 1 - Shooter Toggle (Square)
        if (gamepad1.square && !prevG1Square) {
            robot.shooter.toggle();
        }
        prevG1Square = gamepad1.square;

        // Gamepad 2 - Schimbare Alianță (L1 = RED, R1 = BLUE)
        if (gamepad2.left_bumper && !prevG2LB) {
            Alliance.current = Alliance.RED;
        }
        if (gamepad2.right_bumper && !prevG2RB) {
            Alliance.current = Alliance.BLUE;
        }
        prevG2LB = gamepad2.left_bumper;
        prevG2RB = gamepad2.right_bumper;

        // Gamepad 2 - Reset Heading Field-Centric (Triangle)
        if (gamepad2.triangle && !prevG2Triangle) {
            robot.drivetrain.resetHeading();
        }
        prevG2Triangle = gamepad2.triangle;

        // Gamepad 2 - Ajustare Putere Shooter (DPad Up / Down)
        if (gamepad2.dpad_up && !prevG2DUp) {
            robot.shooter.increasePower(0.05);
        }
        if (gamepad2.dpad_down && !prevG2DDown) {
            robot.shooter.decreasePower(0.05);
        }
        prevG2DUp = gamepad2.dpad_up;
        prevG2DDown = gamepad2.dpad_down;

        // Telemetrie OpMode
        telemetry.addData("Alliance", Alliance.current);
        telemetry.addData("Feedforward kF", F_GAIN);
        telemetry.addData("Tolerance (deg)", TOLERANCE_DEGREES);
        telemetry.addData("Vision Alpha", VISION_ALPHA);
        telemetry.addData("Limelight Target", robot.vision != null && robot.vision.hasTarget());
        if (robot.vision != null && robot.vision.hasTarget()) {
            telemetry.addData("Limelight Raw Tx", robot.vision.getTargetX());
            telemetry.addData("Limelight Filtered Tx", robot.vision.getFilteredTargetX(VISION_ALPHA));
            telemetry.addData("Limelight Target Y (ty)", robot.vision.getTargetY());
        }

        // Transmisie Unică Telemetrie la Finalul Buclei
        robot.updateTelemetry();
    }
}
