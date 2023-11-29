package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class KhepriTeleOp extends LinearOpMode {
	KhepriBot robot;
	GamepadEvents controller1;
	double drive, strafe, rotate, liftPos, liftPower, climbPos, climbPower, loop, prevTime, normalizedPowerMultiplier, heading, target, error;
	boolean headingLock, climbAboveHeight, climbBelowHeight;

	final double maxClimbHeight = 14384;

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
//					heading );


			if( controller1.x.onPress( ) ) {
				headingLock = !headingLock;
				if( headingLock )
					target = heading;
			}

			drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast() : KhepriBot.DriveSpeeds.DRIVE.getNorm());
			strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast() : KhepriBot.DriveSpeeds.STRAFE.getNorm());
			rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast() : KhepriBot.DriveSpeeds.ROTATE.getNorm());

			if( headingLock ) {
				error = findShortestAngularTravel( target, heading );
				rotate += robot.headingController.calculate( error, 0 );
			}

			robot.drive.drive( drive, strafe, rotate );

			liftPos = robot.lift.getPosition();
			liftPower = gamepad1.right_trigger - (gamepad1.left_trigger * 0.6);

			robot.lift.setPower( liftPower );

			climbPos = robot.climber.getPosition();
			climbPower = robot.climber.getPower();

			climbAboveHeight = climbPos > maxClimbHeight;
			climbBelowHeight = climbPos < 0;


			if( controller1.a.onPress( ) )
				robot.deposit.releaseToggle( );

			if( controller1.b.onPress() )
				robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );


			if ( liftPos < 50 && liftPower < 0 )
				robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else if (liftPower > 0 && liftPos > 90)
				robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );

			if(controller1.left_bumper.onPress())
				robot.intake.foldIntake();
			else if( controller1.right_bumper.onPress() )
				robot.intake.deployIntake( normalizedPowerMultiplier );

			if ( controller1.dpad_up.onPress() && !climbAboveHeight )
				robot.climber.goUp();
			else if( controller1.dpad_down.onPress() && !climbBelowHeight )
				robot.climber.goDown();

			if (climbAboveHeight && climbPower > 0 || climbBelowHeight && climbPower < 0 )
				robot.climber.setPower( 0 );

			if (gamepad1.options)
				robot.launcher.release();

			displayTelemetry();
			controller1.update( );
			robot.clearBulkCache();
		}
	}

	public void displayTelemetry() {
		telemetry.addData( "lift pos", liftPos );
		telemetry.addData( "release pos", robot.deposit.getReleasePosition( ) );
		telemetry.addData( "angler pos", robot.deposit.getAnglePosition( ) );
		telemetry.addData( "STATE", robot.drive.getWheelState( ) );
		telemetry.addData( "current draw total", robot.getRobotCurrentAmps() );
		telemetry.addData( "launcher position", robot.launcher.getPosition() );
		telemetry.addData( "climb encoder", robot.climber.getPosition() );
		loop = System.currentTimeMillis( );
		telemetry.addData( "hz ", 1000 / (loop - prevTime) );
		prevTime = loop;
		robot.drive.displayWheelAngles( telemetry );

		telemetry.update( );
	}
}
