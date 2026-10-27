package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.configurables.FindingBallConfig;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.PDController;

public class VisionAlignCommand {
    private final Robot robot;
    private final PDController pdController;

    public VisionAlignCommand(Robot robot, PDController pdController) {
        this.robot = robot;
        this.pdController = pdController;
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
        if (robot.vision != null && robot.vision.hasTarget()) {
            double error = robot.vision.getFilteredTargetX();
            double turn = pdController.calculate(error, FindingBallConfig.TOLERANCE_DEGREES);
            robot.drivetrain.drive(forward, strafe, turn, false);
        } else {
            robot.drivetrain.drive(forward, strafe, 0.0, false);
        }
    }

    public boolean isAligned() {
        if (robot.vision != null && robot.vision.hasTarget()) {
            return pdController.isWithinTolerance(robot.vision.getFilteredTargetX());
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
                .setDone(this::isAligned)
                .setEnd(endCondition -> stop());
    }
}
