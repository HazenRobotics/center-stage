package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    Servo retractionServo;
    Servo rotationServo;      ervo;

    public Outtake() {
        retractionServo = new Servo(hardwareMap.get(Servo.class, "retractionServo"));
        rotationServo = new Servo(hardwareMap.get(Servo.class, "rotationServo"));
    }
}
