package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;

//@Photon
@TeleOp
@Config
public class KhepriTeleOp extends LinearOpMode {

	KhepriBot robot;
	GamepadEvents controller1, controller2;
	double drive, strafe, rotate, liftPos,
			liftPower, climbPos, climbPower = 0,
			numLoops, heading, targetHeading, error,
			currentHz, averageHz, loop, prevTime, totalHz, targetX, targetY;
	ElapsedTime positionLockSettlingTimer;
	Pose2D poseEstimate;
	boolean headingLock, wasHeadingLocked, positionLock, positionSettling, climbAboveHeight = false, climbBelowHeight = false,
			endGame = false, releaseAutoControl = true, usingLiftPID, wasUsingPID;
	final double MAX_CLIMB_HEIGHT = 14384, CONTROLLER_TOLERANCE = 0.01;

	RevBlinkinLedDriver.BlinkinPattern[] signallingColors = new RevBlinkinLedDriver.BlinkinPattern[2];
	int colorIndex = 0;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupTeleOpTracker( new Pose2D( 0, 0, new AngleDegrees( 0 ) ) );
		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );
		telemetry.setMsTransmissionInterval( 100 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
		positionLockSettlingTimer = new ElapsedTime( );

		waitForStart( );
		positionLockSettlingTimer.reset( );

		while( opModeIsActive( ) ) {
			calculateHz();
			endGamePolling();

			driveControl( );
			liftControl();
			depositControl();
			intakeControl();
			climbControl();
			launcherControl();
			LEDControl();

			if(controller1.start.onPress()) robot.tracker.resetHeading( );

			controller1.update( );
			controller2.update( );
			robot.update();
			displayTelemetry( );
		}
	}

	public void driveControl( ) {
		poseEstimate = robot.getPose( );
		heading = poseEstimate.getTheta( ).getRadians( );

		drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast( ) : KhepriBot.DriveSpeeds.DRIVE.getNorm( ));
		strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast( ) : KhepriBot.DriveSpeeds.STRAFE.getNorm( ));
		rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast( ) : KhepriBot.DriveSpeeds.ROTATE.getNorm( ));

		headingLock = Math.abs(rotate) < CONTROLLER_TOLERANCE;
		positionLock = Math.abs(drive) + Math.abs(strafe) < CONTROLLER_TOLERANCE && headingLock;

		if( headingLock ) {
			if (!wasHeadingLocked) targetHeading = heading;
			error = findShortestAngularTravel( targetHeading, heading );
			rotate += robot.teleOpHeadingController.calculate( error, 0 );
		}

		positionSettling = positionLockSettlingTimer.seconds() < 0.5;

		if( positionLock ) {
			if (positionSettling){
				targetY = poseEstimate.getY();
				targetX = poseEstimate.getX();
			} else {
				drive += robot.YController.calculate( poseEstimate.getY( ), targetY );
				strafe += robot.XController.calculate( poseEstimate.getX( ), targetX );
			}
		} else positionLockSettlingTimer.reset( );

		wasHeadingLocked = headingLock;

		robot.drive.fieldCentricDrive( drive, strafe, rotate, heading );
	}

	public void climbControl() {
		if( controller2.dpad_up.onPress( ) && !climbAboveHeight ) {
			endGame = true;
			robot.climber.goUp( );
		} else if( controller2.dpad_down.onPress( ) && !climbBelowHeight ) {
			endGame = true;
			robot.climber.goDown( );
		}

		if( (climbAboveHeight && climbPower > 0) || (climbBelowHeight && climbPower < 0) ) robot.climber.setPower( 0 );
	}

	public void launcherControl() {
		if( gamepad2.right_stick_button ) robot.launcher.release( );
	}

	public void intakeControl() {
		if( controller1.right_bumper.onPress( ) ) {
			robot.intake.deployIntake( KhepriBot.normalizedPowerMultiplier );
			robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
			robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
		} else if( controller1.left_bumper.onPress( ) ) robot.intake.foldIntake( );

		if (robot.intake.isReversed()) gamepad1.rumble( 100 );
	}

	public void depositControl() {
		if( controller1.a.onPress( ) ) robot.deposit.releaseToggle( );
		if( controller1.b.onPress( ) ) robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );

		if( controller1.x.onPress( ) ) {
			releaseAutoControl = !releaseAutoControl;
			if (releaseAutoControl) robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
			else robot.deposit.setAngleState( Deposit.AngleStates.FIX_BUCKET );
		}

		if (releaseAutoControl) {
			if( liftPos < 50 && liftPower < 0 && robot.deposit.getReleaseState() == Deposit.ReleaseStates.RETRACTED ) robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
			else if( liftPower > 0 && liftPos > 90 ) robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
		}
	}

	public void endGamePolling() {
		if( endGame ) {
			climbPos = robot.climber.getPosition( );
			climbPower = robot.climber.getPower( );
			climbAboveHeight = climbPos > MAX_CLIMB_HEIGHT;
			climbBelowHeight = climbPos < 0;
		}
	}

	public void liftControl() {
		liftPos = robot.lift.getPosition();
		liftPower = (gamepad1.right_trigger - (gamepad1.left_trigger * 0.7));
		usingLiftPID = Math.abs(liftPower) < CONTROLLER_TOLERANCE;

		if (usingLiftPID && liftPos > 10) {
			if (!wasUsingPID) robot.lift.setTarget( (int) liftPos );

			robot.lift.updatePID();
		} else {
			if (wasUsingPID) robot.lift.setTarget( (int) liftPos );
			robot.lift.setPower( liftPower );
		}
		wasUsingPID = usingLiftPID;
	}

	public void LEDControl() {
		if (controller2.a.onPress()) signallingColors[colorIndex++] = RevBlinkinLedDriver.BlinkinPattern.GREEN;
		if (controller2.b.onPress()) signallingColors[colorIndex++] = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
		if (controller2.y.onPress()) signallingColors[colorIndex++] = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
		if (controller2.x.onPress()) signallingColors[colorIndex++] = RevBlinkinLedDriver.BlinkinPattern.WHITE;

		if ( colorIndex == 2 ) {
			telemetry.addLine("Writing to driver");
			colorIndex = 0;
			robot.rgbController.writeColors( signallingColors );
		}
	}

	public void calculateHz() {
		//more sporadic hz calculation
		currentHz = robot.getCurrentHz();

		//other method because more stability
		numLoops++;
		totalHz += currentHz;
		averageHz = totalHz / numLoops;
	}

	public void displayTelemetry( ) {
		telemetry.addData( "lift pos", liftPos );
		telemetry.addData( "release pos", robot.deposit.getReleasePosition( ) );
		telemetry.addData( "angler pos", robot.deposit.getAnglePosition( ) );
		telemetry.addData( "intake angle", robot.intake.getAngle( ) );
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
		telemetry.addData( "current hz", currentHz );
		telemetry.addData( "average hz", averageHz );

		telemetry.addData( "colorIndex", colorIndex );

		telemetry.addData( "color1", signallingColors[0] );
		telemetry.addData( "color2", signallingColors[1] );
		telemetry.addData( "color1Controller", robot.rgbController.getRgbArray()[0] );
		telemetry.addData( "color2Controller", robot.rgbController.getRgbArray()[1] );
		telemetry.addData( "rgbcontrollerTimer seconds", robot.rgbController.getTimer().seconds() );
		telemetry.addData( "rgbcontrollerTimer updateRotation", robot.rgbController.getUpdateRotation() );

//		robot.drive.displayWheelAngles( telemetry );
		telemetry.update( );
	}
}
