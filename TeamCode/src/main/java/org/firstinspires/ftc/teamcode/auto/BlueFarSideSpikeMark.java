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
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueFarSideSpikeMark extends LinearOpMode {

	KhepriBot robot;
	VisionPortal visionPortal;
	PropProcessor propProcessor;
	PropProcessor.PropPosition position;

	public enum AutoState {
		DRIVE_TO_MARK,
		PLACE_MARK,
		PARK,
		PARKED
	}

	AutoState autoState;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.deposit.setReleasePosition( Deposit.ReleaseStates.EXTENDED );
		robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );

		propProcessor = new PropProcessor();

		propProcessor.setPropColor( PropProcessor.PropColor.BLUE );

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get( WebcamName.class, "Webcam 1"))
				.addProcessor(propProcessor)
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.YUY2)
				.setAutoStopLiveView(true)
				.build();

		while( opModeInInit() && !opModeIsActive() ) {
			telemetry.addData( "position", propProcessor.getPiecePosition() );
			telemetry.update();
		}

		position = propProcessor.getPiecePosition();
		autoState = AutoState.DRIVE_TO_MARK;

		robot.roadrunnerDrive.getLocalizer().setPoseEstimate( new Pose2d(-36.5,61.5, Math.toRadians( 270 ) ));

		TrajectorySequence spikeMarkDrive = robot.roadrunnerDrive.trajectorySequenceBuilder(new Pose2d(-36.5,61.5, Math.toRadians( 270 )))
				.addTemporalMarker( () -> robot.lift.setTarget( 100 ) )
				.splineToConstantHeading(new Vector2d( -35, 35 ), Math.toRadians( 270 ))
				.build();

		TrajectorySequence leftSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder(spikeMarkDrive.end())
				.turn( Math.toRadians( 75 ) )
//				.addTemporalMarker( () -> {
//					robot.lift.setTarget( 100 );
//				} )
//				.waitSeconds( 1 )
//				.addTemporalMarker( () -> {
//					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );
//				} )
				.waitSeconds( 3 )
				.addTemporalMarker( () -> {
					robot.deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
				} )
				.turn( Math.toRadians( -75 ) )
				.build();

		TrajectorySequence middleSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder(spikeMarkDrive.end())
				.turn( Math.toRadians( 15 ) )
//				.addTemporalMarker( () -> {
//					robot.lift.setTarget( 100 );
//				} )
//				.waitSeconds( 1 )
//				.addTemporalMarker( () -> {
//					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );
//				} )
				.waitSeconds( 3 )
				.addTemporalMarker( () -> {
					robot.deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
				} )
				.turn( Math.toRadians( -15 ) )
				.build();

		TrajectorySequence rightSpikeMarkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder(spikeMarkDrive.end())
				.turn( Math.toRadians( -15 ) )
//				.addTemporalMarker( () -> {
//					robot.lift.setTarget( 100 );
//				} )
//				.waitSeconds( 1 )
//				.addTemporalMarker( () -> {
//					robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );
//				} )
				.waitSeconds( 3 )
				.addTemporalMarker( () -> {
					robot.deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
				} )
				.turn( Math.toRadians( 15 ) )
				.build();

		TrajectorySequence scoreSpike;

		TrajectorySequence parkTraj = robot.roadrunnerDrive.trajectorySequenceBuilder( new Pose2d(-35, 35 , Math.toRadians( 270 ) ) )
				.addTemporalMarker( () -> robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB ))
				.waitSeconds( 1 )
				.addTemporalMarker( () -> robot.lift.setPower( -0.1 ) )
				.setReversed( true )
				.splineToLinearHeading( new Pose2d( -35, 10, Math.toRadians( 0 ) ), 0 )
				.addTemporalMarker( () -> robot.lift.setPower( 0 ) )
				.splineTo( new Vector2d( 35, 10 ), 0 )
				.splineToConstantHeading( new Vector2d( 55, 10 ), 0 )
				.build();

		robot.roadrunnerDrive.followTrajectorySequenceAsync( spikeMarkDrive );
		waitForStart();

		if(position == PropProcessor.PropPosition.LEFT) scoreSpike = leftSpikeMarkTraj;
		else if(position == PropProcessor.PropPosition.RIGHT) scoreSpike = rightSpikeMarkTraj;
		else scoreSpike = middleSpikeMarkTraj;

		while( opModeIsActive() ) {
			robot.roadrunnerDrive.update();
			robot.lift.updatePID( );

			switch( autoState ) {
				case DRIVE_TO_MARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						robot.roadrunnerDrive.followTrajectorySequence( scoreSpike );
						autoState = AutoState.PLACE_MARK;
					}
					break;
				case PLACE_MARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						robot.roadrunnerDrive.followTrajectorySequenceAsync( parkTraj );
						autoState = AutoState.PARK;
					}
					break;
				case PARK:
					if (!robot.roadrunnerDrive.isBusy()) {
						autoState = AutoState.PARKED;
					}
					break;
			}

		}
	}
}
