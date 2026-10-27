package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurables.FindingBallConfig;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class VisionSubsystem {
    public static final int GOAL_PIPELINE = 0; // AprilTags / Containere
    public static final int BALL_PIPELINE = 1; // Detector Culoare / Bile

    private Limelight3A limelight;
    private LLResult latestResult;
    private Telemetry telemetry;
    private int currentPipeline = GOAL_PIPELINE;

    // EMA Filter State
    private double filteredTx = 0.0;
    private double filteredTy = 0.0;
    private boolean isFilterInitialized = false;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(GOAL_PIPELINE);
                limelight.start();
            }
        } catch (Exception e) {
            limelight = null;
        }
    }

    public void update() {
        if (limelight != null) {
            latestResult = limelight.getLatestResult();
        }

        if (hasTarget()) {
            double rawTx = getTargetX();
            double rawTy = getTargetY();
            double alpha = Math.max(0.01, Math.min(1.0, FindingBallConfig.VISION_ALPHA));

            if (!isFilterInitialized) {
                filteredTx = rawTx;
                filteredTy = rawTy;
                isFilterInitialized = true;
            } else {
                // EMA formula: filtered = alpha * raw + (1 - alpha) * filtered
                filteredTx = alpha * rawTx + (1.0 - alpha) * filteredTx;
                filteredTy = alpha * rawTy + (1.0 - alpha) * filteredTy;
            }
        } else {
            resetFilter();
        }
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTargetX() {
        return hasTarget() ? latestResult.getTx() : 0.0;
    }

    public double getFilteredTargetX() {
        return filteredTx;
    }

    public double getFilteredTargetY() {
        return filteredTy;
    }

    public void resetFilter() {
        filteredTx = 0.0;
        filteredTy = 0.0;
        isFilterInitialized = false;
    }

    public double getTargetY() {
        return hasTarget() ? latestResult.getTy() : 0.0;
    }

    public double getDistanceToTargetCm() {
        if (!hasTarget()) {
            return 0.0;
        }
        double ty = getFilteredTargetY();
        double totalAngleRad = Math.toRadians(FindingBallConfig.CAMERA_PITCH_DEGREES - ty);
        double deltaH = FindingBallConfig.BALL_CENTER_HEIGHT_CM - FindingBallConfig.CAMERA_HEIGHT_CM;
        return Math.abs(deltaH / Math.tan(totalAngleRad));
    }

    public Pose3D getBotPose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    public void setPipeline(int pipelineIndex) {
        if (currentPipeline != pipelineIndex) {
            currentPipeline = pipelineIndex;
            resetFilter();
            if (limelight != null) {
                limelight.pipelineSwitch(pipelineIndex);
            }
        }
    }

    public int getPipeline() {
        return currentPipeline;
    }

    public void switchToBallPipeline() {
        setPipeline(BALL_PIPELINE);
    }

    public void switchToGoalPipeline() {
        setPipeline(GOAL_PIPELINE);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public Command periodic() {
        return infinite(() -> {
            update();
            if (telemetry != null) {
                telemetry.addData("Limelight Target Found", hasTarget());
                telemetry.addData("Limelight Raw Tx", getTargetX());
                telemetry.addData("Limelight Ty", getTargetY());
                telemetry.addData("Limelight Pipeline", currentPipeline);
            }
        });
    }
}
