package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.PDController;

public class VisionAlignCommand {
    private final Robot robot;
    private final PDController pdController;
    private double alpha = 0.15; // Default EMA smoothing factor matching VisionAlignTeleOp.VISION_ALPHA

    public VisionAlignCommand(Robot robot, PDController pdController) {
        this.robot = robot;
        this.pdController = pdController;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double getAlpha() {
        return alpha;
    }

    public void start() {
        pdController.reset();
        if (robot.vision != null) {
            robot.vision.resetFilter();
            robot.vision.setPipeline(VisionSubsystem.BALL_PIPELINE);
        }
    }

    public void execute() {
        execute(0.0, 0.0);
    }

    public void execute(double forward, double strafe) {
        execute(forward, strafe, pdController.getTolerance(), alpha);
    }

    public void execute(double forward, double strafe, double tolerance, double alpha) {
        if (robot.vision != null && robot.vision.hasTarget()) {
            double error = robot.vision.getFilteredTargetX(alpha);
            double turn = pdController.calculate(error, tolerance);
            robot.drivetrain.drive(forward, strafe, turn, false);
        } else {
            robot.drivetrain.drive(forward, strafe, 0.0, false);
        }
    }

    public boolean isAligned() {
        return isAligned(pdController.getTolerance(), alpha);
    }

    public boolean isAligned(double tolerance, double alpha) {
        if (robot.vision != null && robot.vision.hasTarget()) {
            return pdController.isWithinTolerance(robot.vision.getFilteredTargetX(alpha));
        }
        return false;
    }

    public void stop() {
        if (robot.vision != null) {
            robot.vision.setPipeline(VisionSubsystem.GOAL_PIPELINE);
        }
    }

    public Command build() {
        return new CommandBuilder()
                .setStart(this::start)
                .setExecute(this::execute)
                .setDone(() -> false);
    }
}
