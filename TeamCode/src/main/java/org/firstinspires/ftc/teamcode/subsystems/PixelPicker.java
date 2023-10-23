package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//
public class PixelPicker {
   //    Servo clawServo;
       Telemetry telemetry;



       public PixelPicker(HardwareMap hw, Telemetry t) {
           this(hw, t, "grip_servo");
       }

        public PixelPicker(HardwareMap hw, Telemetry t, String servoName) {
            clawServo = hw.get(Servo.class,servoName);
            telemetry = t;
        }

        public void openClaw(double power)
        {
           clawServo.setPosition(power);
           telemetry.addData("ClawStatus: ","Open");
        }

        public void closeClaw(double power)
        {
            clawServo.setPosition(power);
            telemetry.addData("ClawStatus: ", "Closed");
        }
}
