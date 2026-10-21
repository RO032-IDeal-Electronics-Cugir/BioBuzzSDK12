package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Robot Auto Command-Based", group = "Autonomous")
public class AutoOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        Follower follower = robot.drivetrain.getFollower();
        if (follower != null) {
            follower.setPose(new Pose(0, 0, 0));
        }

        Path path1 = line(new Pose(0, 0, 0), new Pose(24, 0, 0)).constant(0);

        waitForStart();

        // Programăm comanda autonomă principală folosind structura Ivy (sequential, parallel, follow, waitMs, instant)
        schedule(sequential(
                parallel(
                        instant(() -> robot.intake.setPower(1.0)),
                        follow(follower, path1)
                ),
                instant(() -> robot.intake.stop()),
                instant(() -> robot.shooter.setPower(0.85)),
                waitMs(2000),
                instant(() -> robot.shooter.setPower(0))
        ));

        // Bucla principală care rulează Scheduler-ul Ivy
        while (opModeIsActive()) {
            Scheduler.execute();
            robot.update();
        }
    }
}
