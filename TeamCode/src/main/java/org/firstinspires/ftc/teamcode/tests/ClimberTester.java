package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

@TeleOp
public class ClimberTester extends LinearOpMode {

	Climber climb;
	@Override
	public void runOpMode( ) throws InterruptedException {
		climb = new Climber( hardwareMap );

		waitForStart();

		while (opModeIsActive()) {
			climb.setPower( gamepad1.right_trigger - gamepad1.left_trigger );

			telemetry.addData( "power that's supposed to be set", gamepad1.right_trigger - gamepad1.left_trigger );
			telemetry.addData( "power that's being set", climb.getPower() );
			telemetry.addData( "current", climb.getCurrent() );
			telemetry.update();
		}
	}
}
