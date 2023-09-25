package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Config
@TeleOp
public class BallLauncherTest extends LinearOpMode {

    DcMotor motor;
    GamepadEvents controller1;
    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get( "motor" );
        controller1 = new GamepadEvents( gamepad1 );
        power = 0;

        waitForStart();

        while(opModeIsActive()) {

            motor.setPower( power );

            controller1.update();
        }
    }
}
