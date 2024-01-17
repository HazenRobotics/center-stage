package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Disabled
public class CRServoTest extends LinearOpMode {
	CRServo servo;
	double power;
	@Override
	public void runOpMode( ) throws InterruptedException {
		servo = hardwareMap.get( CRServo.class, "wheelServo ");

		waitForStart();

		while(opModeIsActive()) {
			if( gamepad1.a )
				power = 1;
			else if( gamepad1.b )
				power = -1;
			else if( gamepad1.y )
				power = 0;

			servo.setPower( power );

			telemetry.addData( "power", power );
			telemetry.update();
		}
	}
}
