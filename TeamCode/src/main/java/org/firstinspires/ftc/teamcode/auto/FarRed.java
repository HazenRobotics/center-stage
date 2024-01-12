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
public class FarRed extends LinearOpMode {

	KhepriBot robot;
	//	RedPropProcessor redPropProcessor;
	VisionPortal visionPortal;

	enum AutoStates {
		INIT_SCANNING,
		SPIKE_MARK_SCORING,
		INITIAL_PIXEL_GRAB,
		INITIAL_BACKDROP_SCORING,
		CYCLING,
		PARK
	}

	enum SpikeMarkScoringStates {
		DRIVE_TO_SCORING_POS,
		SCORE,
	}

	enum InitialPickupStates {
		DRIVE_TO_STACK,
		GRAB_FROM_STACK,
		DRIVE_AWAY_AND_EJECT,
	}

	enum BackdropScoringStates {
		DRIVE_TO_BACKDROP_YELLOW,
		SCORE_ON_BACKDROP_YELLOW,
		SHIFT_OVER_WHITE,
		SCORE_ON_BACKDROP_WHITE
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
	InitialPickupStates pickupStates = InitialPickupStates.DRIVE_TO_STACK;
	BackdropScoringStates backdropState = BackdropScoringStates.DRIVE_TO_BACKDROP_YELLOW;
	CyclingStates cyclingStates = CyclingStates.DRIVE_TO_STACK;

	RedPropProcessor.PropPosition position;

	Pose2D leftMark, middleMark, rightMark, selectedMark, stackLocation, ejectPoint,
			middleInitialPixel, leftInitialPixel, selectedInitialPixel;
	GVFPath leftToStack, middleToStack, rightToStack, selectedStack, selectedBackdrop,
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
		robot.setupAutoTracker( new Pose2D( -39.5, -63.5, new AngleDegrees( 90 ) ) );
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

		leftMark = new Pose2D( -48, -12, 90 );
		middleMark = new Pose2D( -36, -12, 90 );
		rightMark = new Pose2D( -36, -36, 180 );
		stackLocation = new Pose2D( -60, -14, 0 );
		ejectPoint = new Pose2D( -50, -14, 0 );
		leftInitialPixel = new Pose2D( 49, -29, 0 );
		middleInitialPixel = new Pose2D( 49, -36, 0 );


		leftToStack = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 49, -29 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( -60, -14 )
				)
		);

		middleToStack = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 49, -36 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( -60, -14 )
				)
		);

		cycleToLeft = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 44, -10 ),
						new Vector2( 56, -11 ),
						new Vector2( 48, -27 )
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

		cycleToMiddle = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 72, 2 ),
						new Vector2( 13, -37 ),
						new Vector2( 48, -36 )
				)
		);

		cycleToRight = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 80, 4 ),
						new Vector2( 34, -25 ),
						new Vector2( 48, -45 )
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
			selectedBackdrop = cycleToLeft;
			selectedStack = leftToStack;
			selectedCycle = cycleToMiddle;
			selectedInitialPixel = middleInitialPixel;
		} else if (position == RedPropProcessor.PropPosition.MIDDLE) {
			selectedMark = middleMark;
			selectedBackdrop = cycleToMiddle;
			selectedStack = middleToStack;
			selectedCycle = cycleToLeft;
			selectedInitialPixel = leftInitialPixel;
		} else if (position == RedPropProcessor.PropPosition.RIGHT) {
			selectedMark = rightMark;
			selectedBackdrop = cycleToRight;
			selectedStack = rightToStack;
			selectedCycle =  cycleToMiddle;
			selectedInitialPixel = leftInitialPixel;
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
							if( position == RedPropProcessor.PropPosition.LEFT ) robot.intake.setIntakeMotorPower( -1 * KhepriBot.normalizedPowerMultiplier );
							else if ( position == RedPropProcessor.PropPosition.MIDDLE ) robot.intake.setIntakeMotorPower( -0.75 * KhepriBot.normalizedPowerMultiplier );
							else if ( position == RedPropProcessor.PropPosition.RIGHT ) robot.intake.setIntakeMotorPower( -0.5 * KhepriBot.normalizedPowerMultiplier );

							if( timer.seconds( ) > 2 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setDeployPos( Intake.DeploymentState.FOLDED );
								robot.lift.setTarget( 125 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								timer.reset( );
								autoState = AutoStates.CYCLING;
							}
							break;
					}
					break;
				case INITIAL_PIXEL_GRAB:
					switch( pickupStates ) {
						case DRIVE_TO_STACK:
							robot.goToPoint( stackLocation );
							if( timer.seconds() > 2 ) {
								robot.intake.setDeployPos( 0.38 );
								robot.intake.setIntakeMotorPower( 0.8 );
								robot.intake.setIntakeServoPower( 0.8 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								timer.reset( );
								pickupStates = InitialPickupStates.GRAB_FROM_STACK;
							}
							break;
						case GRAB_FROM_STACK:
							robot.goToPoint( stackLocation );
							if( timer.seconds( ) > 3 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setIntakeServoPower( 0 );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
								timer.reset( );
								pickupStates = InitialPickupStates.DRIVE_AWAY_AND_EJECT;;
							}
							break;
						case DRIVE_AWAY_AND_EJECT:
							robot.goToPoint( ejectPoint, 0.5, 1 );
							if( timer.seconds( ) > 0.1 ) {
								robot.intake.setIntakeMotorPower( -1 );
								robot.intake.setIntakeServoPower( -1 );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
							}
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
								autoState = AutoStates.INITIAL_BACKDROP_SCORING;
							}
							break;
					}
					break;
				case INITIAL_BACKDROP_SCORING:
					switch(backdropState) {
						case DRIVE_TO_BACKDROP_YELLOW:
							robot.followPath( selectedBackdrop, 0 );
							if( selectedBackdrop.getPathState() == GVFPath.PathState.DONE ) {
								timer.reset( );
								robot.deposit.setReleaseState( Deposit.ReleaseStates.DROP_ONE );
								backdropState = BackdropScoringStates.SCORE_ON_BACKDROP_YELLOW;
							}
							break;
						case SCORE_ON_BACKDROP_YELLOW:
							robot.followPath( selectedBackdrop, 0 );
							if( timer.seconds( ) > 0.5 ) {
								timer.reset( );
								backdropState = BackdropScoringStates.SHIFT_OVER_WHITE;
							}
							break;
						case SHIFT_OVER_WHITE:
							robot.goToPoint( selectedInitialPixel );
							if( selectedBackdrop.getPathState() == GVFPath.PathState.DONE ) {
								timer.reset( );
								robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
								backdropState = BackdropScoringStates.SCORE_ON_BACKDROP_YELLOW;
							}
							break;
						case SCORE_ON_BACKDROP_WHITE:
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
								robot.intake.setDeployPos( 0.38 );
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
							robot.goToPoint( ejectPoint, 0.5, 1 );
							if( timer.seconds( ) > 0.1 ) {
								robot.intake.setIntakeMotorPower( -1 );
								robot.intake.setIntakeServoPower( -1 );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
							}
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
							break;
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
							robot.followPath( selectedCycle, 0 );
							if( timer.seconds( ) > 0.5 ) robot.deposit.setReleaseState( Deposit.ReleaseStates.DROP_ONE );
							if( timer.seconds( ) > 1.0 )  robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );
							if( timer.seconds( ) > 1.25 ) robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
							if( timer.seconds( ) > 1.5 ) {
								robot.lift.setTarget( 0 );
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
