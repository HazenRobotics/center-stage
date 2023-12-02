package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

@TeleOp
public class KhepriTeleOp extends LinearOpMode {

	KhepriBot robot;
	GamepadEvents controller1, controller2;
	double drive, strafe, rotate, liftPos, liftPower, climbPos, climbPower = 0, numLoops, normalizedPowerMultiplier, heading, target, error;
	ElapsedTime timer;
	Pose2D poseEstimate;
	boolean headingLock, climbAboveHeight = false, climbBelowHeight = false;
	boolean endGame = false, releaseAutoControl = true;
	final double maxClimbHeight = 14384;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupTracker( new Pose2D( 0, 0, new AngleDegrees( 0 ).getTheta() ) );
		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );
		telemetry.setMsTransmissionInterval( 20 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
		timer = new ElapsedTime( );

		waitForStart( );
		timer.reset( );

		while( opModeIsActive( ) ) {
			normalizedPowerMultiplier = 12.0 / robot.hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS );

			endGamePolling();

			driveControl( );
			liftControl();
			depositControl();
			intakeControl();
			climbControl();
			launcherControl();

			if(controller1.start.onPress()) {
				robot.tracker.resetHeading();
			}

			displayTelemetry( );
			controller1.update( );
			controller2.update( );
			robot.update();
		}
	}

	public void displayTelemetry( ) {
		telemetry.addData( "lift pos", liftPos );
		telemetry.addData( "release pos", robot.deposit.getReleasePosition( ) );
		telemetry.addData( "angler pos", robot.deposit.getAnglePosition( ) );
//		telemetry.addData( "STATE", robot.drive.getWheelState( ) );
//		telemetry.addData( "heading", heading );
//		telemetry.addData( "climb dir", robot.climber.getDirection() );
//
//		if( endGame ) {
//			telemetry.addData( "climb power", climbPower );
//			telemetry.addData( "climb pos", climbPos );
//			telemetry.addData( "above height", climbAboveHeight );
//			telemetry.addData( "below height", climbBelowHeight );
//		}
//		telemetry.addData( "current draw total", robot.getRobotCurrentAmps() );
//		telemetry.addData( "climb encoder", robot.climber.getPosition() );
//		telemetry.addData( "intake current draw", robot.intake.getMotorCurrent() );
//		telemetry.addData( "intake servo power", robot.intake.getIntakeServoPower() );
		telemetry.addData( "pose", poseEstimate );
		numLoops++;
		telemetry.addData( "hz ", numLoops / timer.seconds( ) );
//		robot.drive.displayWheelAngles( telemetry );

		telemetry.update( );
	}

	public void driveControl( ) {
		poseEstimate = robot.getPose( );
		heading = poseEstimate.getTheta( ).getRadians( );

		if( controller1.y.onPress( ) ) {
			headingLock = !headingLock;
			if( headingLock )
				target = heading;
		}

		drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast( ) : KhepriBot.DriveSpeeds.DRIVE.getNorm( ));
		strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast( ) : KhepriBot.DriveSpeeds.STRAFE.getNorm( ));
		rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast( ) : KhepriBot.DriveSpeeds.ROTATE.getNorm( ));

		if( headingLock ) {
			error = findShortestAngularTravel( target, heading );
			rotate += robot.headingController.calculate( error, 0 );
		}

		robot.drive.fieldCentricDrive( drive, strafe, rotate, heading - (Math.PI / 2) );
	}

	public void climbControl() {
		if( controller2.dpad_up.onPress( ) && !climbAboveHeight ) {
			endGame = true;
			robot.climber.goUp( );
		} else if( controller2.dpad_down.onPress( ) && !climbBelowHeight ) {
			endGame = true;
			robot.climber.goDown( );
		}

		if( (climbAboveHeight && climbPower > 0) || (climbBelowHeight && climbPower < 0) )
			robot.climber.setPower( 0 );
	}

	public void launcherControl() {
		if( gamepad2.options ) {
			endGame = true;
			robot.launcher.release( );
		}
	}

	public void intakeControl() {
		if( controller1.right_bumper.onPress( ) ) {
			robot.intake.deployIntake( normalizedPowerMultiplier );
			robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED );
			robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
		} else if( controller1.left_bumper.onPress( ) )
			robot.intake.foldIntake( );

		if (controller1.dpad_up.onPress()) {
			robot.intake.adjustUp();
		} else if (controller1.dpad_down.onPress()) {
			robot.intake.adjustDown();
		} else if (controller1.dpad_right.onPress()) {
			robot.intake.setDeployPos( Intake.DeploymentState.TOP_TWO );
		}
	}

	public void depositControl() {
		if( controller1.a.onPress( ) ) robot.deposit.releaseToggle( );

		if( controller1.x.onPress( ) ) {
			releaseAutoControl = !releaseAutoControl;
			if (releaseAutoControl) robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else robot.deposit.setAnglePosition( Deposit.AngleStates.FIX_BUCKET );
		}
		if (releaseAutoControl) {
			if( liftPos < 50 && liftPower < 0 )
				robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else if( liftPower > 0 && liftPos > 90 )
				robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
		}
	}

	public void endGamePolling() {
		if( endGame ) {
			climbPos = robot.climber.getPosition( );
			climbPower = robot.climber.getPower( );
			climbAboveHeight = climbPos > maxClimbHeight;
			climbBelowHeight = climbPos < 0;
		}
	}

	public void liftControl() {
		liftPos = robot.lift.getPosition( );
		liftPower = gamepad1.right_trigger - (gamepad1.left_trigger * 0.6);

		robot.lift.setPower( liftPower );
	}
}
