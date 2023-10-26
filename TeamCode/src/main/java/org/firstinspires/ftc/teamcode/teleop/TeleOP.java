package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pixel_Intaker;


public class TeleOP extends LinearOpMode {
    MecDrive bot = new MecDrive();


    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;
    Claw claw = new Claw(HardwareMap, Telemetry);
    Lift lift = new Lift(HardwareMap, Telemetry);
    Pixel_Intaker pixelIntaker = new Pixel_Intaker(HardwareMap, Telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            if(gamepad1.a){claw.openClaw(0.4);}
            if(gamepad1.b){claw.closeClaw(0.2);}
            lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            //Move joystick in certain ways to move in certain directions(drive, strafe, rotate)
            bot.moveBot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.dpad_up){ pixelIntaker.intakeServoOn(0.5); pixelIntaker.intakeMotorPressed(0.6);};
            if(gamepad1.dpad_down){ pixelIntaker.intakeServoOn(-0.5);};


        }
    }
}
