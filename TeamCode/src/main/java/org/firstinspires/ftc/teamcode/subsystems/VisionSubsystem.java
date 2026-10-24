package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionSubsystem {
    private Limelight3A limelight;
    private LLResult latestResult;

    public VisionSubsystem(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    public void update() {
        if (limelight != null) {
            latestResult = limelight.getLatestResult();
        }
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTargetX() {
        return hasTarget() ? latestResult.getTx() : 0.0;
    }

    public double getTargetY() {
        return hasTarget() ? latestResult.getTy() : 0.0;
    }

    public Pose3D getBotPose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
