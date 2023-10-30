package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    //Make an array because the servo
    DcMotor liftMotor;
    Servo liftServo;
    Telemetry telemetry;
    public Lift(HardwareMap hw, Telemetry telemetry){ this(hw, telemetry, "lift_Motor", "Lift_Servo");}
    public Lift(HardwareMap hw, Telemetry telemetry, String liftName, String servoName) {

        liftMotor = hw.get(DcMotor.class, liftName);
        liftServo = hw.get(Servo.class, servoName);
        this.telemetry = telemetry;
    }


    public void setPower(double power) { liftMotor.setPower(power);}

    public void rotateServo(double position){
        liftServo.setPosition(position);

    }

}
