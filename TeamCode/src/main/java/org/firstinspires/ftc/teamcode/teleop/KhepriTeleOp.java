package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class KhepriTeleOp extends LinearOpMode {
	KhepriBot robot;
	GamepadEvents controller1;
	double liftPos, liftPower, loop, loopTime, normalizedPowerMultiplier;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		controller1 = new GamepadEvents( gamepad1 );

		waitForStart( );

		while( opModeIsActive( ) ) {
			normalizedPowerMultiplier = 12.0 / robot.hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS ) ;

//			heading = robot.imu.getRobotYawPitchRollAngles().getYaw( AngleUnit.RADIANS );

//			if( controller1.x.onPress( ) )
//				robot.drive.setWheelState( robot.drive.getWheelState() == CoaxialSwerveDrive.WheelState.DRIVE ?
//									CoaxialSwerveDrive.WheelState.X : CoaxialSwerveDrive.WheelState.DRIVE );

//			robot.drive.fieldCentricDrive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
//					robot.imu.getRobotYawPitchRollAngles().getYaw( AngleUnit.RADIANS ) );
			robot.drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

			liftPos = robot.lift.getMotorPosition();
			liftPower = gamepad1.right_trigger - (gamepad1.left_trigger * 0.6);

			robot.lift.setPower( liftPower );

			if( controller1.a.onPress( ) )
				robot.deposit.releaseToggle( );

			if ( liftPos < 50 && liftPower < 0 )
				robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else if (liftPower > 0 && liftPos > 90)
				robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );

			if((controller1.left_bumper.onPress() || robot.bucketFull()))
				robot.intake.foldIntake();
			else if( controller1.right_bumper.onPress() )
				robot.intake.deployIntake( normalizedPowerMultiplier );

			if ( controller1.dpad_up.onPress() )
				robot.climber.goUp();
			else if( controller1.dpad_down.onPress() )
				robot.climber.goDown();

			if (gamepad1.options)
				robot.launcher.release();
			//deals with the intake sellting the 2nd pixel
			if(!robot.ramp.getBeamState()) {
				robot.intake.isSettling=true;
			}
			if(!robot.top.getBeamState()&& robot.isIntakeClear()) {
				robot.intake.isSettling=false;
			}
			if(robot.intake.isSettling) {
				robot.intake.setIntakeMotorPower(Intake.SETTLING_POWER);
			}

			displayTelemetry();
			controller1.update( );
		}
	}

	public void displayTelemetry() {
		telemetry.addData( "lift pos", liftPos );
		telemetry.addData( "release pos", robot.deposit.getReleasePosition( ) );
		telemetry.addData( "angler pos", robot.deposit.getAnglePosition( ) );
		telemetry.addData( "STATE", robot.drive.getWheelState( ) );
		telemetry.addData( "current draw total", robot.getRobotCurrentAmps() );
		telemetry.addData( "launcher position", robot.launcher.getPosition() );
		loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;
		robot.drive.displayWheelAngles( telemetry );

		telemetry.update( );

	}
}
