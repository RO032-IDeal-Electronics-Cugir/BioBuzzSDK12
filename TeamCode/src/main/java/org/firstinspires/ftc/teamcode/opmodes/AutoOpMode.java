package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Robot Auto Command-Based", group = "Autonomous")
public class AutoOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        robot = new Robot(hardwareMap);

        Follower follower = robot.drivetrain.getFollower();
        if (follower != null) {
            follower.setPose(new Pose(0, 0, 0));
        }

        Path path1 = line(new Pose(0, 0, 0), new Pose(24, 0, 0)).constant(0);

        waitForStart();

        schedule(sequential(
                parallel(
                        instant(() -> robot.intake.on()),
                        follow(follower, path1)
                ),
                instant(() -> robot.intake.stop()),
                instant(() -> robot.shooter.turnOn()),
                waitMs(2000),
                instant(() -> robot.shooter.turnOff())
        ));

        while (opModeIsActive()) {
            Scheduler.execute();
            robot.update();
        }
    }
}
