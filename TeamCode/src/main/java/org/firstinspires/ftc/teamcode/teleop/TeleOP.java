package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pixel_Intaker;



public class TeleOP extends LinearOpMode {
    MecDrive bot = new MecDrive();
    Enum anEnum = Enum.THING;

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;

    Lift lift = new Lift(HardwareMap, Telemetry);
    Pixel_Intaker pixelIntaker = new Pixel_Intaker(HardwareMap, Telemetry);
    Deposit deposit = new Deposit(HardwareMap);
    int retractPress = 0;
    int rotatePress = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        if(anEnum==Enum.THING) {

        }
        while (opModeIsActive()) {

            lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            //Move joystick in certain ways to move in certain directions(drive, strafe, rotate)
            bot.moveBot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.dpad_up){ pixelIntaker.intakeServoOn(0.5); pixelIntaker.intakeMotorPressed(0.6);};
            if(gamepad1.dpad_down){ pixelIntaker.intakeServoOn(-0.5);};
            if(gamepad1.a) {
                if(retractPress == 0) {
                    //drop first
                    deposit.extendDeposit();
                    retractPress ++;
                } else if(retractPress == 1) {
                    //drop second
                    deposit.dropSecondPixel();
                    retractPress++;
                } else if(retractPress == 2) {
                    //extend
                    deposit.dropFirstPixel();
                    retractPress = 0;
                }
            }
            // The rotation of the deposit should be automatic
            if(gamepad1.b){
                //rotate up
                if(rotatePress == 0){
                    deposit.rotateDepositUp();
                    rotatePress++;
                //rotate down
                }else if(rotatePress == 1){
                    deposit.rotateDepositDown();
                    rotatePress = 0;
                }
            }
        }
    }
}
