package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class DrivetrainSubsystem {
    private static Pose poseTransfer = Pose.zero();

    private Follower follower;
    private Localizer localizer;
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private Telemetry telemetry;

    private boolean isFieldCentric = false;
    private double headingOffset = 0.0;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            follower = null;
        }

        if (follower == null || follower.localizer == null) {
            try {
                localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
            } catch (Exception e) {
                localizer = null;
            }
        }

        try {
            frontLeft = hardwareMap.get(DcMotor.class, "fl");
            backLeft = hardwareMap.get(DcMotor.class, "bl");
            frontRight = hardwareMap.get(DcMotor.class, "fr");
            backRight = hardwareMap.get(DcMotor.class, "br");

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Motors not yet configured in hardware map
        }
    }

    private static double signedSquare(double raw) {
        return Math.signum(raw) * Math.pow(raw, 2);
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
        if (localizer != null) {
            localizer.update();
        }
        poseTransfer = getPose();
    }

    public void toggleDriveMode() {
        isFieldCentric = !isFieldCentric;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.isFieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public void resetHeading() {
        headingOffset = getHeading();
    }

    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, Alliance.current, true);
    }

    public void drive(double forward, double strafe, double turn, boolean applySquareTurn) {
        drive(forward, strafe, turn, Alliance.current, applySquareTurn);
    }

    public void drive(double forward, double strafe, double turn, Alliance alliance, boolean applySquareTurn) {
        if (isFieldCentric) {
            driveFieldCentric(forward, strafe, turn, alliance, applySquareTurn);
        } else {
            driveRobotCentric(forward, strafe, turn, applySquareTurn);
        }
    }

    public void driveRobotCentric(double forward, double strafe, double turn, boolean applySquareTurn) {
        if (frontLeft != null && backLeft != null && frontRight != null && backRight != null) {
            forward = signedSquare(forward);
            strafe = signedSquare(strafe);
            if (applySquareTurn) {
                turn = signedSquare(turn);
            }

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
            double flPower = (forward + strafe + turn) / denominator;
            double blPower = (forward - strafe + turn) / denominator;
            double frPower = (forward - strafe - turn) / denominator;
            double brPower = (forward + strafe - turn) / denominator;

            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);
        }
    }

    public void driveFieldCentric(double forward, double strafe, double turn, Alliance alliance, boolean applySquareTurn) {
        if (frontLeft != null && backLeft != null && frontRight != null && backRight != null) {
            double heading = getHeading() - headingOffset;

            if (alliance == Alliance.BLUE) {
                heading += Math.PI;
            }

            forward = signedSquare(forward);
            strafe = signedSquare(strafe);
            if (applySquareTurn) {
                turn = signedSquare(turn);
            }

            double x = strafe * Math.cos(heading) + forward * Math.sin(heading);
            double y = strafe * -Math.sin(heading) + forward * Math.cos(heading);

            y *= 1.1; // Compensate for mecanum friction

            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1.0);

            frontLeft.setPower((y + x + turn) / denominator);
            frontRight.setPower((y - x - turn) / denominator);
            backLeft.setPower((y - x + turn) / denominator);
            backRight.setPower((y + x - turn) / denominator);
        }
    }

    public Pose getPose() {
        if (follower != null && follower.localizer != null) {
            return follower.localizer.pose();
        }
        if (localizer != null) {
            return localizer.pose();
        }
        return Pose.zero();
    }

    public double getHeading() {
        return getPose().heading();
    }

    public void setStartingPose(Pose pose) {
        if (follower != null) {
            follower.setPose(pose);
        }
        if (localizer != null) {
            localizer.setPose(pose);
        }
    }

    public void usePreviousStartingPose() {
        setStartingPose(poseTransfer);
    }

    public Command periodic() {
        return infinite(() -> {
            update();
            if (telemetry != null) {
                Pose currentPose = getPose();
                telemetry.addData("Robot X", currentPose.x());
                telemetry.addData("Robot Y", currentPose.y());
                telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading()));
                telemetry.addData("Drive Mode", isFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            }
        });
    }

    public boolean isBusy() {
        return follower != null && follower.isBusy();
    }

    public Follower getFollower() {
        return follower;
    }

    public Localizer getLocalizer() {
        return localizer != null ? localizer : (follower != null ? follower.localizer : null);
    }
}
