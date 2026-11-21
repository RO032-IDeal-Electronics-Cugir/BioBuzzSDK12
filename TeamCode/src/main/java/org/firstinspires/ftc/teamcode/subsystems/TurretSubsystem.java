package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.robot.Alliance;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
@Configurable
public class TurretSubsystem {

    public enum TelemetryLevel {
        NONE,
        MATCH,
        DEBUG
    }

    public enum Mode {
        AUTO_AIM,
        LOCK_FORWARD,
        MANUAL
    }

    public enum TargetGoal {
        BOTTOM,
        TOP
    }

    private CRServo turretServo;
    private DcMotor turretEncoder;
    private Telemetry telemetry;
    private Follower follower;
    private DrivetrainSubsystem drivetrain;

    private Mode mode = Mode.AUTO_AIM;
    private TargetGoal currentGoal = TargetGoal.BOTTOM;
    private Alliance alliance = Alliance.RED;
    private boolean isOpModeActive = false;

    private double manualTargetAngleDeg = 0.0;
    private double lastError = 0.0;
    private long lastTimeNano = System.nanoTime();
    private double currentDistanceInches = 0.0;
    private double currentTargetAngleDeg = 0.0;
    private double currentServoPower = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null, null);
    }

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        this.telemetry = telemetry;
        this.follower = follower;

        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretEncoder = hardwareMap.get(DcMotor.class, "bl");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public void setDrivetrain(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setOpModeActive(boolean active) {
        this.isOpModeActive = active;
    }

    public boolean isOpModeActive() {
        return isOpModeActive;
    }

    public TargetGoal getCurrentGoal() {
        return currentGoal;
    }

    public double getCurrentDistanceInches() {
        return currentDistanceInches;
    }

    public double getCurrentTargetAngleDeg() {
        return currentTargetAngleDeg;
    }

    public double getTurretAngleDeg() {
        double ticks = turretEncoder.getCurrentPosition() * (TurretConfig.INVERT_ENCODER ? -1.0 : 1.0);
        return (ticks * 360.0) / TurretConfig.TICKS_PER_TURRET_REV;
    }

    public void setManualTargetAngleDeg(double angleDeg) {
        this.manualTargetAngleDeg = Math.max(TurretConfig.MIN_LIMIT_DEG,
                                    Math.min(TurretConfig.MAX_LIMIT_DEG, angleDeg));
    }

    public void adjustManualAngle(double deltaDeg) {
        setManualTargetAngleDeg(manualTargetAngleDeg + deltaDeg);
    }

    public boolean resetEncoder() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return true;
    }

    public Command periodic() {
        return infinite(() -> {
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - lastTimeNano) / 1e9;
            if (dt <= 0) dt = 0.001;
            lastTimeNano = currentTimeNano;

            // Pauza de siguranță la Init (Fără mișcare în faza de pregătire)
            if (!isOpModeActive && !TurretConfig.ENABLED_IN_INIT) {
                turretServo.setPower(0.0);
                lastError = 0.0;
                return;
            }

            // Preluare pozitie robot de la Pedro Pathing / Localizer Fallback
            Pose robotPose = (drivetrain != null)
                    ? drivetrain.getPose()
                    : ((follower != null && follower.localizer != null) ? follower.localizer.pose() : Pose.zero());

            // Calcul pozitie turela cu decalajul de -10 cm (-3.937 inchi)
            double offsetInches = TurretConfig.TURRET_OFFSET_Y_CM / 2.54;
            double turretX = robotPose.x() + offsetInches * Math.cos(robotPose.heading());
            double turretY = robotPose.y() + offsetInches * Math.sin(robotPose.heading());

            // Selecție dinamică coș (Sus vs. Jos) cu histerezis
            double robotY = robotPose.y();
            if (currentGoal == TargetGoal.BOTTOM && robotY > (TurretConfig.MIDLINE_Y + TurretConfig.HYSTERESIS_MARGIN)) {
                currentGoal = TargetGoal.TOP;
            } else if (currentGoal == TargetGoal.TOP && robotY < (TurretConfig.MIDLINE_Y - TurretConfig.HYSTERESIS_MARGIN)) {
                currentGoal = TargetGoal.BOTTOM;
            }

            // Determinare coordonate țintă pe teren
            double goalX = (alliance == Alliance.RED)
                    ? (currentGoal == TargetGoal.BOTTOM ? TurretConfig.RED_GOAL_BOTTOM_X : TurretConfig.RED_GOAL_TOP_X)
                    : (currentGoal == TargetGoal.BOTTOM ? TurretConfig.BLUE_GOAL_BOTTOM_X : TurretConfig.BLUE_GOAL_TOP_X);

            double goalY = (alliance == Alliance.RED)
                    ? (currentGoal == TargetGoal.BOTTOM ? TurretConfig.RED_GOAL_BOTTOM_Y : TurretConfig.RED_GOAL_TOP_Y)
                    : (currentGoal == TargetGoal.BOTTOM ? TurretConfig.BLUE_GOAL_BOTTOM_Y : TurretConfig.BLUE_GOAL_TOP_Y);

            // Calcul distanta pana la tinta
            currentDistanceInches = Math.hypot(goalX - turretX, goalY - turretY);

            // Calcul unghi dorit al turelei
            double targetAngleDeg = 0.0;
            switch (mode) {
                case AUTO_AIM:
                    double angleToGoalRad = Math.atan2(goalY - turretY, goalX - turretX);
                    double robotHeadingRad = robotPose.heading();
                    double relativeAngleRad = angleToGoalRad - robotHeadingRad;
                    targetAngleDeg = Math.toDegrees(relativeAngleRad);
                    break;
                case LOCK_FORWARD:
                    targetAngleDeg = 0.0;
                    break;
                case MANUAL:
                    targetAngleDeg = manualTargetAngleDeg;
                    break;
            }

            double currentAngleDeg = getTurretAngleDeg();

            // Smart Cable Unwinding bazat pe limitele configurate (MIN_LIMIT_DEG / MAX_LIMIT_DEG)
            double delta = Math.IEEEremainder(targetAngleDeg - currentAngleDeg, 360.0);
            double candidateAngleDeg = currentAngleDeg + delta;

            if (candidateAngleDeg > TurretConfig.MAX_LIMIT_DEG) {
                if (candidateAngleDeg - 360.0 >= TurretConfig.MIN_LIMIT_DEG) {
                    targetAngleDeg = candidateAngleDeg - 360.0;
                } else {
                    targetAngleDeg = TurretConfig.MAX_LIMIT_DEG;
                }
            } else if (candidateAngleDeg < TurretConfig.MIN_LIMIT_DEG) {
                if (candidateAngleDeg + 360.0 <= TurretConfig.MAX_LIMIT_DEG) {
                    targetAngleDeg = candidateAngleDeg + 360.0;
                } else {
                    targetAngleDeg = TurretConfig.MIN_LIMIT_DEG;
                }
            } else {
                targetAngleDeg = candidateAngleDeg;
            }

            currentTargetAngleDeg = targetAngleDeg;

            // Calcul Controller PD pentru Axon CRServo
            double error = targetAngleDeg - currentAngleDeg;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double power = 0.0;
            if (Math.abs(error) > TurretConfig.TOLERANCE_DEG) {
                power = TurretConfig.P_GAIN * error + TurretConfig.D_GAIN * derivative;
                power = Math.max(-1.0, Math.min(1.0, power));
            }

            if (TurretConfig.INVERT_SERVO) {
                power = -power;
            }

            currentServoPower = power;
            turretServo.setPower(power);
        });
    }

    public void printTelemetry(Telemetry telemetry, TelemetryLevel level) {
        if (telemetry == null || level == TelemetryLevel.NONE) return;

        telemetry.addData("Turret Mode", mode);
        telemetry.addData("Turret Target Goal", currentGoal);

        if (level == TelemetryLevel.DEBUG) {
            telemetry.addData("Turret Alliance", alliance);
            telemetry.addData("Turret Target Angle", String.format(java.util.Locale.US, "%.1f deg", currentTargetAngleDeg));
            telemetry.addData("Turret Current Angle", String.format(java.util.Locale.US, "%.1f deg", getTurretAngleDeg()));
            telemetry.addData("Turret Raw Ticks", turretEncoder.getCurrentPosition());
            telemetry.addData("Turret Servo Power", String.format(java.util.Locale.US, "%.2f", currentServoPower));
            telemetry.addData("Turret Distance to Goal", String.format(java.util.Locale.US, "%.1f in", currentDistanceInches));
        }
    }
}
