package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    Servo retractionServo;
    Servo rotationServo;
    HardwareMap hardwareMap;

    public Deposit(HardwareMap hw) {
        hardwareMap=hw;
        retractionServo = hw.get(Servo.class, "retractionServo");
        rotationServo = hw.get(Servo.class, "rotationServo");
    }


    public void rotateDepositUp() {
        rotationServo.setPosition(1.0);
    }
    public void rotateDepositDown() {
        rotationServo.setPosition(0.0);
    }

    public void extendDeposit() {
        retractionServo.setPosition(1.0);
    }
    public void dropFirstPixel() {
        retractionServo.setPosition(0.5);
    }

    public void dropSecondPixel() {
        retractionServo.setPosition(0.0);
    }
}
