package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PDController;

public class VisionAlignCommand {
    private final Robot robot;
    private final PDController pdController;

    public VisionAlignCommand(Robot robot, PDController pdController) {
        this.robot = robot;
        this.pdController = pdController;
    }

    public void execute() {
        if (robot.vision != null && robot.vision.hasTarget()) {
            double error = robot.vision.getTargetX();
            double turn = pdController.calculate(error);
            robot.drivetrain.drive(0, 0, turn);
        } else {
            robot.drivetrain.drive(0, 0, 0);
        }
    }

    public Command build() {
        return new CommandBuilder()
                .setStart(() -> {
                    pdController.reset();
                    if (robot.vision != null) {
                        robot.vision.setPipeline(1);
                    }
                })
                .setExecute(this::execute)
                .setDone(() -> false);
    }
}
