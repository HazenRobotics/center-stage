package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class FarRed extends LinearOpMode {

	KhepriBot robot;
	PropProcessor propProcessor;
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
		KNOCK_OFF_STACK,
		PICKUP_PIXEL_FROM_GROUND,
	}

	enum BackdropScoringStates {
		DRIVE_TO_BACKDROP_YELLOW,
		SCORE_ON_BACKDROP_YELLOW,
		SHIFT_OVER_WHITE,
		SCORE_ON_BACKDROP_WHITE
	}

	enum CyclingStates {
		DRIVE_TO_STACK,

		KNOCK_OFF_STACK,
		PICKUP_PIXEL_FROM_GROUND,
		DRIVE_AWAY_AND_EJECT,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP
	}

	AutoStates autoState = AutoStates.INIT_SCANNING;
	SpikeMarkScoringStates spikeState = SpikeMarkScoringStates.DRIVE_TO_SCORING_POS;
	InitialPickupStates pickupStates = InitialPickupStates.DRIVE_TO_STACK;
	BackdropScoringStates backdropState = BackdropScoringStates.DRIVE_TO_BACKDROP_YELLOW;
	CyclingStates cyclingStates = CyclingStates.DRIVE_TO_STACK;

	PropProcessor.PropPosition position;
	Pose2D leftMark, middleMark, rightMark, selectedMark, stackLocation, ejectPoint,
			middleInitialPixel, leftInitialPixel, rightInitialPixel, selectedWhitePixel, pickupFromGroundPoint;
	GVFPath leftToStack, middleToStack, rightToStack, selectedStack, selectedBackdrop,
			cycleToLeft, cycleToMiddle, cycleToRight, selectedCycle, rightToBackdrop;

	@Override
	public void runOpMode( ) throws InterruptedException {
		//if right, go forward, rotate right, deliver pixel, go to common point to go under truss
		//go under truss, drive to score pixel on back, park on left side

		//if left, go forward, rotate left, deliver pixel,
		//go under truss, drive to score pixel on back, park on left side

		//if middle, go to common point, rotate 180,
		//drop pixel, go under drive to score pixel on back, park on left side

		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( -38.5, -63.5, new AngleDegrees( 90 ) ) );
		robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
		robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
		robot.drive.setMaxSpeed( 0.6 );
//		robot.lift.setTarget( 50 );

		propProcessor = new PropProcessor().setPropColor( PropProcessor.PropColor.RED_FAR );

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get( WebcamName.class, "front"))
				.addProcessor( propProcessor )
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.enableLiveView( true )
				.setAutoStopLiveView(true)
				.build();

		leftMark = new Pose2D( -46, -18, 90 );
		middleMark = new Pose2D( -36, -14, 90 );
		rightMark = new Pose2D( -36, -36, 180 );
		stackLocation = new Pose2D( -58, -14, 0 );
		ejectPoint = new Pose2D( -50, -14, 0 );
		pickupFromGroundPoint = new Pose2D( -53, -14, 0 );
		leftInitialPixel = new Pose2D( 49, -29, 0 );
		middleInitialPixel = new Pose2D( 49, -36, 0 );
		rightInitialPixel = new Pose2D( 49, -45, 0 );

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
						new Vector2( 49, -29 )
				)
		);

		cycleToMiddle = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -61, -14 ),
						new Vector2( 72, 2 ),
						new Vector2( 13, -37 ),
						new Vector2( 49, -36 )
				)
		);

		cycleToRight = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -60, -14 ),
						new Vector2( 80, 4 ),
						new Vector2( 34, -25 ),
						new Vector2( 49, -45 )
				)
		);

		rightToBackdrop = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -36, -36 ),
						new Vector2( -80, -2 ),
						new Vector2( 50, 8 ),
						new Vector2( 49, -45 )
				)
		);

		while( opModeInInit( ) && !opModeIsActive( ) ) {
			position = propProcessor.getPropPosition();
			robot.drive.drive( 0, 0.01, 0 );
			telemetry.addData( "position", position );
			telemetry.update();
			robot.update();
		}

		telemetry.addLine( "cycles inbound" );
		telemetry.update();

		waitForStart( );

		ElapsedTime timer = new ElapsedTime( );
		ElapsedTime autoStopwatch = new ElapsedTime( );

		if (position == PropProcessor.PropPosition.LEFT) {
			selectedMark = leftMark;
			selectedBackdrop = cycleToLeft;
			selectedStack = leftToStack;
			selectedCycle = cycleToMiddle;
			selectedWhitePixel = middleInitialPixel;
		} else if (position == PropProcessor.PropPosition.MIDDLE) {
			selectedMark = middleMark;
			selectedBackdrop = cycleToMiddle;
			selectedStack = middleToStack;
			selectedCycle = cycleToLeft;
			selectedWhitePixel = leftInitialPixel;
		} else if (position == PropProcessor.PropPosition.RIGHT) {
			selectedMark = rightMark;
			selectedBackdrop = rightToBackdrop;
			selectedStack = rightToStack;
			selectedCycle =  cycleToMiddle;
			selectedWhitePixel = middleInitialPixel;
		}

		robot.intake.setDeployPos( 0.5 );
		visionPortal.stopStreaming( );

		while( opModeIsActive( ) ) {
			switch( autoState ) {
				case INIT_SCANNING:
					autoState = AutoStates.SPIKE_MARK_SCORING;
					break;
				case SPIKE_MARK_SCORING:
					switch( spikeState ) {
						case DRIVE_TO_SCORING_POS:
							robot.goToPoint( selectedMark );
							if( timer.seconds( ) > 2.5 ) {
								timer.reset( );
								spikeState = SpikeMarkScoringStates.SCORE;
							}
							break;
						case SCORE:
							robot.goToPoint( selectedMark );

							if( timer.seconds( ) < 2 ) {
								if( position == PropProcessor.PropPosition.LEFT ) robot.intake.setIntakeMotorPower( -0.75 * KhepriBot.normalizedPowerMultiplier );
								else if ( position == PropProcessor.PropPosition.MIDDLE ) robot.intake.setIntakeMotorPower( -0.75 * KhepriBot.normalizedPowerMultiplier );
								else if ( position == PropProcessor.PropPosition.RIGHT ) robot.intake.setIntakeMotorPower( -0.5 * KhepriBot.normalizedPowerMultiplier );
							} else {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setDeployPos( Intake.DeploymentState.FOLDED );
//								timer.reset( );
							}

							if( timer.seconds( ) > 12 ) autoState = AutoStates.INITIAL_BACKDROP_SCORING;
							break;
					}
					break;
//				case INITIAL_PIXEL_GRAB:
//					switch( pickupStates ) {
//						case DRIVE_TO_STACK:
//							robot.goToPoint( stackLocation );
//							if( timer.seconds() > 2 ) {
//								robot.intake.setDeployPos( 0.357 );
//								robot.intake.setIntakeMotorPower( 0.9 );
//								robot.intake.setIntakeServoPower( 0.9 );
//								timer.reset( );
//								pickupStates = InitialPickupStates.KNOCK_OFF_STACK;
//							}
//							break;
//						case KNOCK_OFF_STACK:
//							robot.goToPoint( stackLocation );
//							if( timer.seconds( ) > 1.5 ) {
//								timer.reset( );
//								pickupStates = InitialPickupStates.PICKUP_PIXEL_FROM_GROUND;;
//							}
//							break;
//						case PICKUP_PIXEL_FROM_GROUND:
//							robot.goToPoint( pickupFromGroundPoint, 0.5, 1 );
//							if( robot.getPose( ).getX( ) > -54 ) robot.intake.setDeployPos( Intake.DeploymentState.FULLY_DEPLOYED );
//
//							if( timer.seconds( ) > 2.5 ) {
//								robot.intake.setIntakeMotorPower( 0 );
//								robot.intake.setIntakeServoPower( 0 );
//								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
//								robot.lift.setTarget( 0 );
//								robot.intake.foldIntake();
//								timer.reset();
//								autoState = AutoStates.INITIAL_BACKDROP_SCORING;
//							}
//							break;
//					}
//					break;
				case INITIAL_BACKDROP_SCORING:
					switch(backdropState) {
						case DRIVE_TO_BACKDROP_YELLOW:
							robot.followPath( selectedBackdrop, 0 );

							if( robot.getPose( ).getX( ) > 12 ) robot.lift.setTarget( 125 );
							if( robot.lift.getPosition( ) > 90 ) robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );

							if( timer.seconds() > 6 ) {
								timer.reset( );
								backdropState = BackdropScoringStates.SCORE_ON_BACKDROP_YELLOW;
							}
							break;
						case SCORE_ON_BACKDROP_YELLOW:
							robot.goToPoint( selectedBackdrop.getEndPoint().getX() + 1, selectedBackdrop.getEndPoint().getY(), 0 );

							if (timer.seconds( ) > 1) robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED );

							if( timer.seconds( ) > 3 ) {
								timer.reset( );
								robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
								robot.lift.setTarget( 0 );
//								backdropState = BackdropScoringStates.SHIFT_OVER_WHITE;
//								autoState = AutoStates.CYCLING;
							}
							break;
//						case SHIFT_OVER_WHITE:
//							robot.goToPoint(selectedWhitePixel );
//							if( timer.seconds( ) > 1 ) {
//								timer.reset( );
//								robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
//								backdropState = BackdropScoringStates.SCORE_ON_BACKDROP_YELLOW;
//							}
//							break;
//						case SCORE_ON_BACKDROP_WHITE:
//							robot.goToPoint( selectedWhitePixel );
//							if( timer.seconds( ) > 0.5 ) robot.deposit.setAngleState( Deposit.AngleStates.GRAB );
//
//							if( timer.seconds( ) > 1.5 ) {
//								robot.lift.setTarget( 0 );
//								robot.intake.foldIntake( );
//								timer.reset( );
//								autoState = AutoStates.PARK;
//							}
//							break;
					}
					break;
				case CYCLING:
					switch( cyclingStates ) {
						case DRIVE_TO_STACK:
							robot.followPath( selectedStack, 0 );
							if( selectedStack.getPathState( ) == GVFPath.PathState.DONE ) {
								robot.intake.setDeployPos( 0.3 );
								robot.intake.setIntakeMotorPower( 0.9 );
								robot.intake.setIntakeServoPower( 0.9 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								timer.reset( );
								cyclingStates = CyclingStates.KNOCK_OFF_STACK;
							}
							break;
						case KNOCK_OFF_STACK:
							robot.followPath( selectedStack, 0 );
							if( timer.seconds( ) > 2 ) {
								timer.reset( );
								cyclingStates = CyclingStates.PICKUP_PIXEL_FROM_GROUND;
							}
							break;
						case PICKUP_PIXEL_FROM_GROUND:
							robot.goToPoint( pickupFromGroundPoint, 0.5, 1 );
							if( robot.getPose( ).getX( ) > -54 ) robot.intake.setDeployPos( Intake.DeploymentState.FULLY_DEPLOYED );

							if( timer.seconds( ) > 2.5 ) {
								robot.intake.setIntakeMotorPower( 0 );
								robot.intake.setIntakeServoPower( 0 );
								robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
								robot.lift.setTarget( 0 );
								robot.intake.foldIntake();
								timer.reset();
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
								autoState = AutoStates.PARK;
							}
							break;
					}
					break;
				case PARK:
					robot.goToPoint( 46, -12, 0 );
					break;
			}

//			if (autoStopwatch.seconds() > 25 && cyclingStates != CyclingStates.SCORE_ON_BACKDROP && cyclingStates != CyclingStates.DRIVE_TO_BACKDROP  ) autoState = AutoStates.PARK;

			TelemetryPacket packet = new TelemetryPacket();
//			Canvas field = packet.fieldOverlay( )
//					.drawImage( "/dash/centerstage.webp", 0, 0, 144, 144, Math.toRadians( 180 ), 72, 72, false )
//					.setAlpha( 1.0 )
//					.drawGrid( 0, 0, 144, 144, 7, 7 )
//					.setRotation( Math.toRadians( 270 ) );

			Pose2D pose = robot.getPose();
//
//			double x = pose.getX();
//			double y = pose.getY();
//			double heading = pose.getTheta().getRadians();
//
//			int robotRadius = 8;
//			field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
//			double arrowX = new Rotation2d(heading).getCos() * robotRadius, arrowY = new Rotation2d(heading).getSin() * robotRadius;
//			double x1 = x, y1 = y;
//			double x2 = x + arrowX, y2 = y + arrowY;
//			field.strokeLine(x1, y1, x2, y2);


//			switch( robot.getDriveControlState() ) {
//				case GVF:
//					GVFPath path = robot.getCurrentPath();
//					if (path != null) {
//						Vector2 firstPoint = path.getCurve( ).getP0( );
//
//						for( double i = 0; i <= 1; i += 0.1 ) {
//							Vector2 secondPoint = path.getCurve( ).calculate( i );
//							field.strokeLine( firstPoint.getX( ), firstPoint.getY( ), secondPoint.getX( ), secondPoint.getY( ) );
//							firstPoint = secondPoint;
//						}
//					}
//					break;
//				case P2P:
//					Vector2 targetPoint = robot.getCurrentPointTarget();
//					if (targetPoint != null) field.strokeLine( robot.getPose().getX( ), robot.getPose().getY( ), targetPoint.getX( ), targetPoint.getY( ) );
//					break;
//			}

			packet.put( "autoState", autoState );
			packet.put( "spikeState", spikeState );
			packet.put( "pickupState", pickupStates );
			packet.put( "backdropState", backdropState );
			packet.put( "cycleState", cyclingStates );
			packet.put( "timer seconds", timer.seconds( ) );
			packet.put( "pose", pose );

			FtcDashboard.getInstance().sendTelemetryPacket(packet);

			telemetry.update();
			robot.update( );



//			if (robot.getPose().getX() > 30 && robot.getPose().getX() < 48) {
//				Point3 pose = aprilTagUtil.getPositionBasedOnTag();
//
//				if (!pose.equals( new Point3( 0, 0, 0 ) ))
//					robot.tracker.setPose2D( new Pose2D( pose.x, pose.y, robot.tracker.getPose2D().getTheta()) );
//			}

			robot.lift.updatePID( );
		}
	}
}
