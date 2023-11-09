package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
public class LiftTest extends LinearOpMode {
	Lift lift;
	@Override
	public void runOpMode( ) throws InterruptedException {
		lift = new Lift( hardwareMap );

		waitForStart();

		while( opModeIsActive() ) {
			lift.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
		}
	}
}
