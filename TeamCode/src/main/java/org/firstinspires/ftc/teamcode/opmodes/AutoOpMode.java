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
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Robot Auto Command-Based", group = "Autonomous")
public class AutoOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        robot = new Robot(hardwareMap);

        schedule(
                robot.drivetrain.periodic(),
                robot.intake.periodic(),
                robot.shooter.periodic(),
                robot.vision.periodic(),
                robot.turret.periodic(),
                robot.periodic()
        );

        Follower follower = robot.drivetrain.getFollower();
        Pose startPose = new Pose(
                TurretConfig.RED_START_X,
                TurretConfig.RED_START_Y,
                Math.toRadians(TurretConfig.RED_START_HEADING_DEG)
        );

        if (follower != null) {
            follower.setPose(startPose);
        }

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b || gamepad2.left_bumper) {
                robot.setAlliance(Alliance.RED);
                startPose = new Pose(
                        TurretConfig.RED_START_X,
                        TurretConfig.RED_START_Y,
                        Math.toRadians(TurretConfig.RED_START_HEADING_DEG)
                );
                if (follower != null) follower.setPose(startPose);
            } else if (gamepad1.x || gamepad2.right_bumper) {
                robot.setAlliance(Alliance.BLUE);
                startPose = new Pose(
                        TurretConfig.BLUE_START_X,
                        TurretConfig.BLUE_START_Y,
                        Math.toRadians(TurretConfig.BLUE_START_HEADING_DEG)
                );
                if (follower != null) follower.setPose(startPose);
            }

            telemetry.addData("Selected Alliance", robot.getAlliance());
            telemetry.addData("Start Pose", startPose);
            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        robot.isAutoShooterAdjustmentEnabled = true;
        robot.turret.setOpModeActive(true);

        Path path1 = line(startPose, new Pose(startPose.x() + 24, startPose.y(), startPose.heading())).constant(startPose.heading());

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
        }
    }
}
