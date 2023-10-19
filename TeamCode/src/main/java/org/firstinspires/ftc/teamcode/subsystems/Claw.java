package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//
public class Claw {
       Servo clawServo;
       Telemetry telemetry;

       public Claw(HardwareMap hw, Telemetry t) {
           this(hw, t, "grip_servo");
       }

        public Claw(HardwareMap hw, Telemetry t, String servoName) {
            clawServo = hw.get(Servo.class,servoName);
            telemetry = t;
        }

        public void openClaw()
        {
           clawServo.setPosition(0.6);
           telemetry.addData("ClawStatus: ","Open");
        }

        public void closeClaw()
        {
            clawServo.setPosition(0.2);
            telemetry.addData("ClawStatus: ", "Closed");
        }
}
