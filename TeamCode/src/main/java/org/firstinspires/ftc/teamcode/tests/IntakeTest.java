package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class IntakeTest extends LinearOpMode {
	KhepriBot robot;
	GamepadEvents controller;
	ElapsedTime timer;
	double intakePower = 0;
	double normalizedPowerMultiplier;


	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.intake.setAdjustIncrement( 0.005 );
		controller = new GamepadEvents( gamepad1 );
		timer = new ElapsedTime( );

		waitForStart();

		while( opModeIsActive() ) {

			// 0.235 top pixel
			// 0.185 bottom pixel

			if( controller.left_bumper.onPress() )
				robot.intake.adjustUp( );
			else if( controller.right_bumper.onPress() )
				robot.intake.adjustDown( );

			if (controller.a.onPress())
				robot.intake.setDeployPos( Intake.DeploymentState.TOP_TWO.getPosition() );

			if(controller.left_bumper.onPress() && timer.seconds() < 0.25)
				robot.intake.foldIntake();

			if(controller.left_bumper.onRelease())
				timer.reset();

			if (controller.x.onPress()) {
				intakePower = 0.8;
			}
			if( controller.y.onPress() ) {
				intakePower = 0;
			}

			normalizedPowerMultiplier = 12.0 / robot.hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS ) ;

			robot.intake.setIntakeMotorPower( intakePower * normalizedPowerMultiplier );
			robot.intake.setIntakeServoPower( intakePower * normalizedPowerMultiplier );

//			robot.intake.addTelemetry();
			telemetry.addData( "power", intakePower );
			telemetry.addData( "timer", timer.seconds() );
			telemetry.addData( "onPress", controller.left_bumper.onPress() );
			telemetry.addData( "onRelease", controller.left_bumper.onRelease() );
			telemetry.update();
			controller.update();
		}
	}
}
