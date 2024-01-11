package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class CloseRed extends LinearOpMode {

	KhepriBot robot;
	//	RedPropProcessor redPropProcessor;
	VisionPortal visionPortal;

	enum AutoStates {
		INIT_SCANNING,
		SPIKE_MARK_SCORING,
		INITIAL_BACKDROP_SCORING,
		CYCLING,
		PARK
	}

	enum SpikeMarkScoringStates {
		DRIVE_TO_SCORING_POS,
		SCORE,
	}

	enum BackdropScoringStates {
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP
	}

	enum CyclingStates {
		DRIVE_TO_STACK,
		GRAB_FROM_STACK,
		DRIVE_AWAY_AND_EJECT,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP
	}

	AutoStates autoState = AutoStates.INIT_SCANNING;
	SpikeMarkScoringStates spikeState = SpikeMarkScoringStates.DRIVE_TO_SCORING_POS;
	BackdropScoringStates backdropState = BackdropScoringStates.DRIVE_TO_BACKDROP;
	CyclingStates cyclingStates = CyclingStates.DRIVE_TO_STACK;

	RedPropProcessor.PropPosition position;

	Pose2D leftMark, middleMark, rightMark, selectedMark,
			leftBackdrop, middleBackdrop, rightBackdrop, selectedBackdrop, cycleScore, ejectPoint = new Pose2D( -50, -14, 0 );
	GVFPath leftToStack, middleToStack, rightToStack, selectedStack,
			cycleToLeft, cycleToMiddle, cycleToRight, selectedCycle;

	@Override
	public void runOpMode( ) throws InterruptedException {
		//if right, go forward, rotate right, deliver pixel, go to common point to go under truss
		//go under truss, drive to score pixel on back, park on left side

		//if left, go forward, rotate left, deliver pixel,
		//go under truss, drive to score pixel on back, park on left side

		//if middle, go to common point, rotate 180,
		//drop pixel, go under drive to score pixel on back, park on left side

		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( 15, -63.5, new AngleDegrees( 90 ) ) );
		robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
		robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
		robot.lift.setTarget( 75 );
//		redPropProcessor = new RedPropProcessor();

//		visionPortal = new VisionPortal.Builder()
//				.setCamera(hardwareMap.get( WebcamName.class, "front"))
//				.addProcessor( redPropProcessor )
//				.setCameraResolution(new Size(640, 480))
//				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//				.setAutoStopLiveView(true)
//				.build();

		position = RedPropProcessor.PropPosition.LEFT;

		leftMark = new Pose2D( 12, -36, 0 );
		middleMark = new Pose2D( 12, -36, 180 );
		rightMark = new Pose2D( 36, -36, 0 );
		leftBackdrop = new Pose2D( 48, -29, 0 );
		middleBackdrop = new Pose2D( 48, -36, 0 );
		rightBackdrop = new Pose2D( 48, -45, 0 );

		leftToStack = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 49, -29 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( -58, -14 )
				)
		);

		middleToStack = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 49, -36 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( -58, -14 )
				)
		);

		rightToStack = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 49, -45 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( -58, -14 )
				)
		);

		cycleToLeft = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( 48, -29 )
				)
		);

		cycleToMiddle = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 72, 2 ),
						new Vector2( 13, -37 ),
						new Vector2( 48, -36 )
				)
		);

//		while( opModeInInit( ) && !opModeIsActive( ) ) {
//			position = redPropProcessor.getPiecePosition();
//			robot.addTelemetryData( "position", position );
//			robot.update();
//		}

		telemetry.addLine( "cycles inbound" );
		telemetry.update();

		waitForStart( );

		ElapsedTime timer = new ElapsedTime( );

		position = RedPropProcessor.PropPosition.LEFT;

		if (position == RedPropProcessor.PropPosition.LEFT) {
			selectedMark = leftMark;
			selectedBackdrop = leftBackdrop;
			selectedStack = leftToStack;
			selectedCycle = cycleToMiddle;
			cycleScore = middleBackdrop;
		} else if (position == RedPropProcessor.PropPosition.MIDDLE) {
			selectedMark = middleMark;
			selectedBackdrop = middleBackdrop;
			selectedStack = middleToStack;
			selectedCycle = cycleToLeft;
			cycleScore = leftBackdrop;
		} else if (position == RedPropProcessor.PropPosition.RIGHT) {
			selectedMark = rightMark;
			selectedBackdrop = rightBackdrop;
			selectedStack = rightToStack;
			selectedCycle =  cycleToMiddle;
			cycleScore = middleBackdrop;
		}

		robot.deposit.setReleaseState( Deposit.ReleaseStates.DROP_ONE );
		robot.intake.setDeployPos( 0.5 );
//		visionPortal.stopStreaming( );

		while( opModeIsActive( ) ) {
			switch( autoState ) {
				case INIT_SCANNING:
					autoState = AutoStates.SPIKE_MARK_SCORING;
					break;
				case SPIKE_MARK_SCORING:
					switch( spikeState ) {
						case DRIVE_TO_SCORING_POS:
							robot.goToPoint( selectedMark );
							if( timer.seconds( ) > 2 ) {
								timer.reset( );
								spikeState = SpikeMarkScoringStates.SCORE;
							}
							break;
						case SCORE:
							robot.goToPoint( selectedMark );
							if( position == RedPropProcessor.PropPosition.RIGHT )
								robot.intake.setIntakeMotorPower( -0.5 * KhepriBot.normalizedPowerMultiplier );
							else
								robot.intake.setIntakeMotorPower( -0.75 * KhepriBot.normalizedPowerMultiplier );
							if( timer.seconds( ) > 2 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setDeployPos( Intake.DeploymentState.FOLDED );
								robot.lift.setTarget( 125 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								timer.reset( );
								autoState = AutoStates.INITIAL_BACKDROP_SCORING;
							}
							break;
					}
					break;
				case INITIAL_BACKDROP_SCORING:
					switch(backdropState) {
						case DRIVE_TO_BACKDROP:
							robot.goToPoint( selectedBackdrop );

							if( timer.seconds( ) > 2.5 ) {
								timer.reset( );
								robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
								backdropState = BackdropScoringStates.SCORE_ON_BACKDROP;
							}
							break;
						case SCORE_ON_BACKDROP:
							robot.goToPoint( selectedBackdrop );
							if( timer.seconds( ) > 0.5 ) {
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
							}
							if( timer.seconds( ) > 1.5 ) {
								robot.lift.setTarget( 0 );
								robot.intake.foldIntake( );
								timer.reset( );
								autoState = AutoStates.CYCLING;
							}
							break;
					}
					break;
				case CYCLING:
					switch( cyclingStates ) {
						case DRIVE_TO_STACK:
							robot.followPath( selectedStack, 0 );
							if( selectedStack.getPathState( ) == GVFPath.PathState.DONE ) {
								robot.intake.setDeployPos( Intake.DeploymentState.FULLY_DEPLOYED );
								robot.intake.setIntakeMotorPower( 0.8 );
								robot.intake.setIntakeServoPower( 0.8 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								timer.reset( );
								cyclingStates = CyclingStates.GRAB_FROM_STACK;
							}
							break;
						case GRAB_FROM_STACK:
							robot.followPath( selectedStack, 0 );
							if( timer.seconds( ) > 3 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setIntakeServoPower( 0 );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
								timer.reset( );
								cyclingStates = CyclingStates.DRIVE_AWAY_AND_EJECT;
							}
							break;
						case DRIVE_AWAY_AND_EJECT:
//							robot.goToPoint( ejectPoint, 0.5, 1 );
							if( timer.seconds( ) > 0.5 ) {
								robot.intake.setIntakeMotorPower( -1 );
								robot.intake.setIntakeServoPower( -1 );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
							}
							if( timer.seconds( ) > 2 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setIntakeServoPower( 0 );
								robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
								timer.reset();
								cyclingStates = CyclingStates.DRIVE_TO_BACKDROP;
							}
						case DRIVE_TO_BACKDROP:
							robot.followPath( selectedCycle, 0 );

							if( robot.getPose( ).getX( ) > 12 ) robot.lift.setTarget( 200 );
							if( robot.lift.getPosition( ) > 100 ) robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );

							if( selectedCycle.getPathState( ) == GVFPath.PathState.DONE ) {
								timer.reset( );
								cyclingStates = CyclingStates.SCORE_ON_BACKDROP;
							}
							break;
						case SCORE_ON_BACKDROP:
							robot.goToPoint( cycleScore );
							if( timer.seconds( ) > 0.5 ) robot.deposit.setReleaseState( Deposit.ReleaseStates.DROP_ONE );
							if( timer.seconds( ) > 1.0 )  robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
							if( timer.seconds( ) > 1.5 ) {
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
								cyclingStates = CyclingStates.DRIVE_TO_STACK;
							}
							break;
					}
					break;
				case PARK:
					robot.goToPoint( 46, -12, 0 );
					break;
			}
			robot.addTelemetryData( "autoState", autoState );
			robot.addTelemetryData( "spikeState", spikeState );
			robot.addTelemetryData( "timer seconds", timer.seconds( ) );
			robot.addTelemetryData( "pose", robot.getPose() );
			telemetry.update();
			robot.update( );
			robot.lift.updatePID( );
		}
	}
}
