package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DrivetrainSubsystem {
    private final Follower follower;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    public boolean isBusy() {
        return follower != null && follower.isBusy();
    }

    public Follower getFollower() {
        return follower;
    }
}
