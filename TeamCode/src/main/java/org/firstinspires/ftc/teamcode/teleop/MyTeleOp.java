package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class MyTeleOp extends OpMode {
    GamepadEvents events = new GamepadEvents(gamepad1);
    Servo servo1 = hardwareMap.servo.get("servo");


    public void init() {

    }

    public void loop() {
        if(events.a.onPress()) {
            if(servo1.getPosition() == 0) {
                servo1.setPosition(1);
            } else {
                servo1.setPosition(0);
            }
        }

        events.update();
    }
}
