package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DrivetrainSubsystem {
    private Follower follower;
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        try {
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            follower = null;
        }

        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

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

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    // TeleOp Mecanum Drive control
    public void drive(double forward, double strafe, double turn) {
        if (frontLeft != null && backLeft != null && frontRight != null && backRight != null) {
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

    public boolean isBusy() {
        return follower != null && follower.isBusy();
    }

    public Follower getFollower() {
        return follower;
    }
}
