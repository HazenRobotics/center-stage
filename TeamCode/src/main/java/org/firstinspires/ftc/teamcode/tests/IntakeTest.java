package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class IntakeTest extends LinearOpMode {
	Intake intake;
	GamepadEvents controller;
	ElapsedTime timer;
	double intakePower = 0;

	@Override
	public void runOpMode( ) throws InterruptedException {
		intake = new Intake( hardwareMap, telemetry );
		controller = new GamepadEvents( gamepad1 );
		timer = new ElapsedTime( );

		waitForStart();

		while( opModeIsActive() ) {

			if( gamepad1.left_bumper )
				intake.adjustUp( );
			else if( gamepad1.right_bumper )
				intake.adjustDown( );

			if(controller.left_bumper.onPress() && timer.seconds() < 0.25)
				intake.foldIntake();

			if(controller.left_bumper.onRelease())
				timer.reset();



			if( controller.a.onPress( ) ) intakePower = intakePower == 1 ? 0 : 1;
			else if( controller.b.onPress( ) ) intakePower = intakePower == -1 ? 0 : -1;

			intake.setIntakeMotorPower( intakePower );

			intake.addTelemetry();
			telemetry.addData( "timer", timer.seconds() );
			telemetry.addData( "onPress", controller.left_bumper.onPress() );
			telemetry.addData( "onRelease", controller.left_bumper.onRelease() );
			telemetry.update();
			controller.update();
		}
	}
}
