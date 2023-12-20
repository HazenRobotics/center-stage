package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class FarBlueStandard extends LinearOpMode {

	KhepriBot robot;
	BluePropProcessor bluePropProcessor;
	VisionPortal visionPortal;
	ElapsedTime timer;

	enum AutoStates {
		INIT_SCANNING,
		SPIKE_MARK_SCORING,
		DRIVE_TO_COMMON_POINT,
		ROTATE_TOWARDS_BACKDROP,
		DRIVE_NEXT_TO_BACKDROP,
		DRIVE_INFRONT_OF_BACKDROP,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP,
		BACK_UP_A_BIT,
		PARK
	}

	enum SpikeMarkScoringStates {
		DRIVE_TO_SCORING_POS,
		ROTATE_TO_SCORE,
		SCORE,
	}

	AutoStates autoState = AutoStates.INIT_SCANNING;
	SpikeMarkScoringStates spikeState = SpikeMarkScoringStates.DRIVE_TO_SCORING_POS;
	BluePropProcessor.PropPosition position;

	@Override
	public void runOpMode( ) throws InterruptedException {
		//if right, go forward, rotate right, deliver pixel, go to common point to go under truss
		//go under truss, drive to score pixel on back, park on left side

		//if left, go forward, rotate left, deliver pixel,
		//go under truss, drive to score pixel on back, park on left side

		//if middle, go to common point, rotate 180,
		//drop pixel, go under drive to score pixel on back, park on left side

		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( -38.5, 63.5, new AngleDegrees( 270 ).getTheta( ) ) );
		robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
		bluePropProcessor = new BluePropProcessor();

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get( WebcamName.class, "front"))
				.addProcessor( bluePropProcessor )
				.setCameraResolution(new Size(640, 480))
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.setAutoStopLiveView(true)
				.build();

		while( opModeInInit( ) && !opModeIsActive( ) ) {
			position = bluePropProcessor.getPiecePosition();
			robot.addTelemetryData( "position", position );
			robot.update();
		}

		waitForStart( );

		ElapsedTime timer = new ElapsedTime( );
		boolean isLeft = position == BluePropProcessor.PropPosition.LEFT;
		boolean isMiddle = position == BluePropProcessor.PropPosition.MIDDLE;
		boolean isRight = position == BluePropProcessor.PropPosition.RIGHT;
		visionPortal.stopStreaming();

		while( opModeIsActive( ) ) {
			switch( autoState ) {
				case INIT_SCANNING:
					autoState = AutoStates.SPIKE_MARK_SCORING;
					break;
				case SPIKE_MARK_SCORING:
					switch( spikeState ) {
						case DRIVE_TO_SCORING_POS:
							if (isRight)
								robot.goToPoint( -46, 18, 270, 1, 1 );
							else if (isMiddle)
								robot.goToPoint( -36, 14, 270, 1, 1 );
							else if (isLeft)
								robot.goToPoint( -36, 36, 270, 1, 1 );
							if( timer.seconds( ) > 4 ) {
								timer.reset( );
								spikeState = SpikeMarkScoringStates.ROTATE_TO_SCORE;
							}
							break;
						case ROTATE_TO_SCORE:
							if (isRight || isMiddle)
								spikeState = SpikeMarkScoringStates.SCORE;
							else if( isLeft )
								robot.goToPoint( -36, 36, 180, 1, 1 );
							if( timer.seconds( ) > 2 ) {
								timer.reset( );
								spikeState = SpikeMarkScoringStates.SCORE;
							}
							break;
						case SCORE:
							if(isLeft)
								robot.intake.setIntakeMotorPower( -0.5 * KhepriBot.normalizedPowerMultiplier );
							else if(isMiddle)
								robot.intake.setIntakeMotorPower( -0.6 * KhepriBot.normalizedPowerMultiplier );
							else if( isRight ) {
								robot.intake.setIntakeMotorPower( -0.75 * KhepriBot.normalizedPowerMultiplier );
							}
							if( timer.seconds( ) > 2 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setDeployPos( Intake.DeploymentState.FOLDED );
								timer.reset( );
								autoState = AutoStates.DRIVE_TO_COMMON_POINT;
							}
							break;
					}
					break;
				case DRIVE_TO_COMMON_POINT:
					if (isRight || isMiddle)
						robot.goToPoint( -40, 12, 270, 1, 1 );
					else if( isLeft )
						robot.goToPoint( -40, 12, 180, 1, 1 );

					if( timer.seconds( ) > 3 ) {
						timer.reset( );
						autoState = AutoStates.ROTATE_TOWARDS_BACKDROP;
					}
					break;
				case ROTATE_TOWARDS_BACKDROP:
					robot.goToPoint( -40, 12, 0, 1, 1 );

					if( timer.seconds( ) > 2 ) {
						timer.reset( );
						autoState = AutoStates.DRIVE_NEXT_TO_BACKDROP;
					}
					break;
				case DRIVE_NEXT_TO_BACKDROP:
					robot.goToPoint( 44, 12, 0 );
					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						robot.lift.setTarget( 175 );
						robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
						robot.deposit.setReleaseState( Deposit.ReleaseStates.HOLD_ONE );
						autoState = AutoStates.DRIVE_TO_BACKDROP;
					}
					break;
				case DRIVE_INFRONT_OF_BACKDROP:
					if( isLeft )
						robot.goToPoint( 44, 45, 0 );
					if( isMiddle )
						robot.goToPoint( 44, 36, 0 );
					if( isRight )
						robot.goToPoint( 44, 29, 0 );
					if( timer.seconds( ) > 2 ) {
						timer.reset( );
						autoState = AutoStates.DRIVE_TO_BACKDROP;
					}
					break;
				case DRIVE_TO_BACKDROP:
					if( isLeft )
						robot.goToPoint( 49, 45, 0 );
					if( isMiddle )
						robot.goToPoint( 49, 36, 0 );
					if( isRight )
						robot.goToPoint( 49, 29, 0 );

					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						autoState = AutoStates.SCORE_ON_BACKDROP;
						robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
					}
					break;
				case SCORE_ON_BACKDROP:
					if( timer.seconds( ) > 1.5 ) {
						robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
						robot.lift.setTarget( 0 );
						timer.reset( );
						autoState = AutoStates.BACK_UP_A_BIT;
					}
					break;
				case BACK_UP_A_BIT:
					if( isLeft )
						robot.goToPoint( 46, 45, 0 );
					if( isMiddle )
						robot.goToPoint( 46, 36, 0 );
					if( isRight )
						robot.goToPoint( 46, 29, 0 );
					if( timer.seconds( ) > 1.5 ) {
						timer.reset( );
						autoState = AutoStates.PARK;
					}
					break;
				case PARK:
					robot.goToPoint( 46, 12, 0 );
					break;
			}
			robot.addTelemetryData( "autoState", autoState );
			robot.addTelemetryData( "spikeState", spikeState );
			robot.addTelemetryData( "isLeft", isLeft );
			robot.addTelemetryData( "isMiddle", isMiddle );
			robot.addTelemetryData( "isRight", isRight );
			robot.addTelemetryData( "timer seconds", timer.seconds() );
			robot.update();
			robot.lift.updatePID();
		}
	}
}
