package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Robot Auto", group = "Autonomous")
public class AutoOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        Follower follower = robot.drivetrain.getFollower();
        if (follower != null) {
            follower.setPose(new Pose(0, 0, 0));
        }

        waitForStart();

        if (isStopRequested()) return;

        // Autonomous routine using subsystems
        robot.intake.setPower(1.0);
        sleep(1000);
        robot.intake.stop();

        robot.shooter.setPower(0.85);
        sleep(2000);
        robot.shooter.stop();
    }
}
