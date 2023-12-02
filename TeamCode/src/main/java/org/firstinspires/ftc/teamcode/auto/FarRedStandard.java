package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class FarRedStandard extends LinearOpMode {

	KhepriBot robot;
	PropProcessor propProcessor;
	VisionPortal visionPortal;
	ElapsedTime timer;

	enum AutoStates {
		INIT_SCANNING,
		DRIVE_TO_SPIKE_MARK,
		ROTATE_TO_SCORE,
		SCORE_SPIKE,
		DRIVE_TO_COMMON_POINT,
		DRIVE_NEXT_TO_BACKDROP,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP,
		BACK_UP_A_BIT,
		PARK
	}

	AutoStates state = AutoStates.INIT_SCANNING;
	PropProcessor.PropPosition position = PropProcessor.PropPosition.LEFT;

	@Override
	public void runOpMode( ) throws InterruptedException {
		//if right, go forward, rotate right, deliver pixel, go to common point to go under truss
		//go under truss, drive to score pixel on back, park on left side

		//if left, go forward, rotate left, deliver pixel,
		//go under truss, drive to score pixel on back, park on left side

		//if middle, go to common point, rotate 180,
		//drop pixel, go under drive to score pixel on back, park on left side

		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupTracker( new Pose2D( -38.5, -63.5, new AngleDegrees( 90 ).getTheta( ) ) );
		robot.deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
//		propProcessor = new PropProcessor()
//		propProcessor.setPropColor( PropProcessor.PropColor.RED );

//		visionPortal = new VisionPortal.Builder()
//				.setCamera(hardwareMap.get( WebcamName.class, "front"))
//				.addProcessor(propProcessor)
//				.setCameraResolution(new Size(640, 480))
//				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//				.setAutoStopLiveView(true)
//				.build();

		while( opModeInInit( ) && !opModeIsActive( ) ) {
//			position = propProcessor.getPiecePosition();
			robot.addTelemetryData( "position", position );
			robot.update();
		}

		waitForStart( );

		ElapsedTime timer = new ElapsedTime( );
		boolean isMiddle = position == PropProcessor.PropPosition.MIDDLE;
//		visionPortal.stopStreaming();

		while( opModeIsActive( ) ) {
			switch( state ) {
				case INIT_SCANNING:
					state = isMiddle ? AutoStates.DRIVE_TO_COMMON_POINT : AutoStates.DRIVE_TO_SPIKE_MARK;
					break;
				case DRIVE_TO_SPIKE_MARK:
					robot.goToPoint( -37, -36, 90 );
					if( timer.seconds( ) > 2 ) {
						timer.reset( );
						state = AutoStates.ROTATE_TO_SCORE;
					}
					break;
				case ROTATE_TO_SCORE:
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( -36, -36, 0 );
					if( position == PropProcessor.PropPosition.RIGHT )
						robot.goToPoint( -36, -36, 180 );
					if( position == PropProcessor.PropPosition.MIDDLE )
						robot.goToPoint( -36, -12, 90 );
					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						state = AutoStates.SCORE_SPIKE;
					}
					break;
				case SCORE_SPIKE:
					robot.intake.setIntakeMotorPower( -0.8 );
					if( timer.seconds( ) > 2 ) {
						robot.intake.setIntakeMotorPower( 0 );
						robot.intake.setDeployPos( Intake.DeploymentState.FOLDED );
						timer.reset( );
						state = isMiddle ? AutoStates.DRIVE_NEXT_TO_BACKDROP : AutoStates.DRIVE_TO_COMMON_POINT;
					}
					break;
				case DRIVE_TO_COMMON_POINT:
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( -36, -12, 0 );
					if( position == PropProcessor.PropPosition.RIGHT )
						robot.goToPoint( -36, -12, 180 );
					if( position == PropProcessor.PropPosition.MIDDLE )
						robot.goToPoint( -36, -12, 90 );
					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						state = AutoStates.DRIVE_NEXT_TO_BACKDROP;
					}
					break;
				case DRIVE_NEXT_TO_BACKDROP:
					robot.goToPoint( 48, -12, 0 );
					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						robot.lift.setTarget( 200 );
						robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
						//					state = AutoStates.DRIVE_TO_BACKDROP;
					}
					break;
				case DRIVE_TO_BACKDROP:
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( 48, -28, 0 );
					if( position == PropProcessor.PropPosition.MIDDLE )
						robot.goToPoint( 48, -36, 0 );
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( 48, -46, 0 );

					if( timer.seconds( ) > 5 ) {
						timer.reset( );
						state = AutoStates.DRIVE_TO_BACKDROP;
					}
					break;
				case SCORE_ON_BACKDROP:
					robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED );
					if( timer.seconds( ) > 1.5 ) {
						timer.reset( );
						state = AutoStates.BACK_UP_A_BIT;
					}
					break;
				case BACK_UP_A_BIT:
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( 46, -28, 0 );
					if( position == PropProcessor.PropPosition.MIDDLE )
						robot.goToPoint( 46, -36, 0 );
					if( position == PropProcessor.PropPosition.LEFT )
						robot.goToPoint( 46, -46, 0 );
					if( timer.seconds( ) > 1.5 ) {
						timer.reset( );
						state = AutoStates.PARK;
					}
					break;
				case PARK:
					robot.goToPoint( 12, -46, 0 );
					break;
			}
			robot.addTelemetryData( "state", state );
			robot.update();
		}
	}
}
