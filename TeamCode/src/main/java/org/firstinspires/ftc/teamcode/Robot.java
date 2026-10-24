package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {
    public DrivetrainSubsystem drivetrain;
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
    }

    public void update() {
        drivetrain.update();
        if (vision != null) {
            vision.update();
        }
    }
}
