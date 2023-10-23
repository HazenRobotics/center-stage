package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.PixelPicker;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;



public class TeleOP extends LinearOpMode {
    MecDrive bot = new MecDrive();


    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;
    PixelPicker claw = new PixelPicker(HardwareMap, Telemetry);
    Lift lift = new Lift(HardwareMap, Telemetry);


    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            if(gamepad1.a){claw.openClaw(0.4);}
            if(gamepad1.b){claw.closeClaw(0.2);}
            lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            //Left joy stick pressed Up, drive forward
            bot.moveBot(gamepad1.left_stick_y, 0, 0);
            bot.moveBot(0, gamepad1.left_stick_x, 0);
            bot.moveBot(0, 0, gamepad1.right_stick_x);


        }
    }
}
