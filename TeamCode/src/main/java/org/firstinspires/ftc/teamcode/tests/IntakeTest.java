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
		intake.setAdjustIncrement( 0.005 );
		controller = new GamepadEvents( gamepad1 );
		timer = new ElapsedTime( );

		waitForStart();

		while( opModeIsActive() ) {

			// 0.235 top pixel
			// 0.185 bottom pixel

			if( controller.left_bumper.onPress() )
				intake.adjustUp( );
			else if( controller.right_bumper.onPress() )
				intake.adjustDown( );

			if (controller.a.onPress())
				intake.setDeployPos( Intake.DeploymentState.TOP_PIXEL.getPosition() );
			else if( controller.b.onPress() )
				intake.setDeployPos( Intake.DeploymentState.SECOND_PIXEL.getPosition() );

			if(controller.left_bumper.onPress() && timer.seconds() < 0.25)
				intake.foldIntake();

			if(controller.left_bumper.onRelease())
				timer.reset();

			intakePower = gamepad1.right_trigger;

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
