package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;


public class Claw {
       Servo clawServo;


       public Claw() {
        clawServo = hardwareMap.servo.get("grip_servo");
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
