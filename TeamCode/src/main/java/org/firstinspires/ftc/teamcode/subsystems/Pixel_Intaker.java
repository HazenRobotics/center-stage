package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pixel_Intaker {
   Servo intakeServo;
   DcMotor intakeMotor;

   Telemetry telemetry;
   public Pixel_Intaker(HardwareMap hw, Telemetry telemetry) { this(hw, telemetry, "intake_Servo", "intake_motor")}
    public Pixel_Intaker(HardwareMap hw, Telemetry telemetry, String servoName, String motorName)
    {//
        intakeServo = hw.get(Servo.class, servoName);
        intakeMotor = hw.get(DcMotor.class, motorName);
        this.telemetry = telemetry;
    }
    public void intakeMotorPressed(double power)
    {

        intakeMotor.setPower(power);
    }
    public void intakeServoOn(double power)
    {
        intakeServo.setPosition(power);
    }



}
