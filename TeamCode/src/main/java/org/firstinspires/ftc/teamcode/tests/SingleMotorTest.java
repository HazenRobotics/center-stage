package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class SingleMotorTest extends LinearOpMode {
	DcMotorEx motor;

	@Override
	public void runOpMode( ) throws InterruptedException {
		motor = hardwareMap.get( DcMotorEx.class, "motor" );

		waitForStart();

		while( opModeIsActive() ) {
			motor.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
		}
	}
}
