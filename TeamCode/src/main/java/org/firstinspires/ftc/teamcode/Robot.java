package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public DrivetrainSubsystem drivetrain;
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;

    public Robot(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
    }

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = null;

        drivetrain = new DrivetrainSubsystem(hardwareMap, null);
        intake = new IntakeSubsystem(hardwareMap, null);
        shooter = new ShooterSubsystem(hardwareMap, null);
        vision = new VisionSubsystem(hardwareMap, null);
    }

    public void update() {
        if (drivetrain != null) {
            drivetrain.update();
        }
        if (vision != null) {
            vision.update();
        }
    }

    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.update();
        }
    }
}
