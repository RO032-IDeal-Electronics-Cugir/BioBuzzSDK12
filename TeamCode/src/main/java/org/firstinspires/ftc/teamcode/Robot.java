package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class Robot {
    public DrivetrainSubsystem drivetrain;
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
    }

    public void update() {
        drivetrain.update();
    }
}
