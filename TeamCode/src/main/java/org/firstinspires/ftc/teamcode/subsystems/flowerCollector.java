package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class flowerCollector {
    private Servo collectorServo;

    public static double ZERO_POSITION = 0.1735;    // 847us
    public static double COLLECT_POSITION = 0.4725; // 1445us

    public flowerCollector(HardwareMap hardwareMap) {
        // Fără try-catch: dacă ceva nu bate în configurație, va apărea eroarea exactă pe Driver Station
        collectorServo = hardwareMap.get(Servo.class, "baraIntake");
        if (collectorServo instanceof PwmControl) {
            ((PwmControl) collectorServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        collectorServo.setPosition(ZERO_POSITION);
    }

    public void goToZero() {
        collectorServo.setPosition(ZERO_POSITION);
    }

    public void collect() {
        collectorServo.setPosition(COLLECT_POSITION);
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("Bara Intake Pos", collectorServo.getPosition());
    }
}