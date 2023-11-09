package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.vision.processors.PixelProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class FarRedSide extends LinearOpMode {

	KhepriBot robot;
	VisionPortal visionPortal;
	PropProcessor propProcessor;

	PropProcessor.PropPosition propPosition;

	public enum AutoState {
		DRIVE_TO_MARK,
		PLACE_MARK,
		CROSS_FIELD,
		SCORE_AND_PARK,
		PARKED
	}

	AutoState autoState;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot(hardwareMap, telemetry);

		robot.deposit.setReleasePosition( Deposit.ReleaseStates.EXTENDED );
		robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );

		propProcessor = new PropProcessor();

		visionPortal = VisionPortal.easyCreateWithDefaults(
				hardwareMap.get(WebcamName.class, "Webcam 1"), propProcessor);

		propProcessor.setPropColor( PropProcessor.PropColor.BLUE );

		TrajectorySequence initialTraj;
		TrajectorySequence leftSpikeMarkTraj, middleSpikeMarkTraj, rightSpikeMarkTraj;
		TrajectorySequence spikeMarkTraj;
		TrajectorySequence crossFieldTraj;
		TrajectorySequence leftBackdropScoreAndParkTraj, middleBackdropScoreAndParkTraj, rightBackdropScoreAndParkTraj;
		TrajectorySequence scoreAndParkTraj;

		initialTraj = robot.roadrunnerDrive.trajectorySequenceBuilder(new Pose2d(-36.5,61.5, Math.toRadians( 270 )))
				.addTemporalMarker( 0, () -> {
					robot.lift.setTarget( 100 );
				} )
				.splineToConstantHeading(new Vector2d( -35, 35 ), Math.toRadians( 270 ))
				.addTemporalMarker( 1, () -> {
					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );
				} )
				.build();

		leftSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( initialTraj.end() )
				.turn( 90 )
				.waitSeconds( 1.5 )
				.build();

		middleSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( initialTraj.end()  )
				.turn( 0 )
				.waitSeconds( 1.5 )
				.build();

		rightSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( initialTraj.end() )
				.turn( -90 )
				.waitSeconds( 1.5 )
				.build();

		crossFieldTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( new Pose2d( -35, 35 ) )
				.addTemporalMarker( () -> {
					robot.deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
				} )
				.splineTo( new Vector2d( -35, 10 ), 0 )
				.splineTo( new Vector2d( 35, 10 ), 0 )
				.build();

		leftBackdropScoreAndParkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( crossFieldTraj.end() )
				.splineToConstantHeading( new Vector2d( 45, 40 ), 0 )
				.addTemporalMarker( () -> {
					robot.lift.setTarget( 400 );
					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
				} )
				.waitSeconds( 2 )
				.forward( 3 )
				.waitSeconds( 0.5 )
				.addTemporalMarker( () -> robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED ) )
				.setReversed( true )
				.splineToConstantHeading( new Vector2d( 59, 10 ), 0 )
				.build();

		middleBackdropScoreAndParkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( crossFieldTraj.end() )
				.splineToConstantHeading( new Vector2d( 40, 40 ), 0 )
				.addTemporalMarker( () -> {
					robot.lift.setTarget( 400 );
					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
				} )
				.waitSeconds( 2 )
				.forward( 3 )
				.waitSeconds( 0.5 )
				.addTemporalMarker( () -> robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED ) )
				.setReversed( true )
				.splineToConstantHeading( new Vector2d( 59, 10 ), 0 )
				.build();

		rightBackdropScoreAndParkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( crossFieldTraj.end() )
				.splineToConstantHeading( new Vector2d( 35, 40 ), 0)
				.addTemporalMarker( () -> {
					robot.lift.setTarget( 400 );
					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
				} )
				.waitSeconds( 2 )
				.forward( 3 )
				.waitSeconds( 0.5 )
				.addTemporalMarker( () -> robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED ) )
				.waitSeconds( 1 )
				.addTemporalMarker( () -> {
					robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB );
					robot.lift.setTarget( 0 );
				} )
				.setReversed( true )
				.splineToConstantHeading( new Vector2d( 59, 10 ), 0 )
				.build();


		robot.roadrunnerDrive.followTrajectorySequenceAsync( initialTraj );
		autoState = AutoState.DRIVE_TO_MARK;

		while( opModeInInit() & !opModeIsActive() ) {
			propPosition = propProcessor.getPiecePosition();
			telemetry.addData( "position", propPosition );
			telemetry.update( );
		}

		waitForStart();
		visionPortal.stopStreaming();

		if(propPosition == PropProcessor.PropPosition.LEFT) {
			spikeMarkTraj = leftSpikeMarkTraj;
			scoreAndParkTraj = leftBackdropScoreAndParkTraj;
		} else if (propPosition == PropProcessor.PropPosition.MIDDLE) {
			spikeMarkTraj = middleSpikeMarkTraj;
			scoreAndParkTraj = middleBackdropScoreAndParkTraj;
		} else {
			spikeMarkTraj = rightSpikeMarkTraj;
			scoreAndParkTraj = rightBackdropScoreAndParkTraj;
		}

		while( opModeIsActive() ) {
			robot.roadrunnerDrive.update();
			robot.lift.updatePID( );

			switch( autoState ) {
				case DRIVE_TO_MARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						robot.roadrunnerDrive.followTrajectorySequenceAsync( spikeMarkTraj );
						autoState = AutoState.PLACE_MARK;
					}
					break;
				case PLACE_MARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						robot.roadrunnerDrive.followTrajectorySequenceAsync( crossFieldTraj );
						autoState = AutoState.CROSS_FIELD;
					}
					break;
				case CROSS_FIELD:
					if (!robot.roadrunnerDrive.isBusy()) {
						robot.roadrunnerDrive.followTrajectorySequenceAsync( scoreAndParkTraj );
						autoState = AutoState.SCORE_AND_PARK;
					}
					break;
				case SCORE_AND_PARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						autoState = AutoState.PARKED;
					}
					break;
			}

		}
	}

	public void displayTelemetry() {
		telemetry.addData( "STATE", autoState );
	}
}
