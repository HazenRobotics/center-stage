package org.firstinspires.ftc.teamcode.auto.twopixel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;

@Autonomous
public class FarBlueUnderTruss extends LinearOpMode {

	KhepriBot robot;
	StateMachine autoMachine;

	public enum AutoStates {
		DRIVE_TO_SPIKE,
		SCORE_SPIKE,
		DETERMINE_RIGHT,
		LEFT_INTERMEDIATE_POINT,
		DRIVE_BEFORE_TRUSS,
		ROTATE,
		DRIVE_UNDER_TRUSS,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP,
		BACKUP,
		PARK,
		RESET_DEPOSIT,
		RESET_LIFT,
		SHUTDOWN,
		TRUSS_FALLBACK1,
		TRUSS_FALLBACK2,
	}

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( -38, 61.125, new AngleDegrees( 270 ) ) );
		robot.setupPropProcessor( PropProcessor.PropColor.BLUE_FAR );
		robot.setMaxAutoDriveSpeed( 0.5 );

		robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
		robot.deposit.setAngleState( Deposit.AngleStates.STRAIGHT_DOWN );
		robot.lift.setTarget( 0 );

//		intake
//		Pose2D leftSpike = new Pose2D( -46, -44, 270 );
//		Pose2D middleSpike = new Pose2D( -40, -31, 270 );
//		Pose2D rightSpike = new Pose2D( -37, -32, 180 );

//		deposit
		GVFPath rightSpike = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -38, 61.125 ),
						new Vector2( -43, 49 ),
						new Vector2( -42.5, 50 ),
						new Vector2( -46, 39 )
				)
		);

		GVFPath middleSpike = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -38, 61.125 ),
						new Vector2( -39, 49 ),
						new Vector2( -39, 46 ),
						new Vector2( -38, 31 )
				)
		);

		GVFPath leftSpike = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -38, 61.125 ),
						new Vector2( -52, 50 ),
						new Vector2( -52, 34 ),
						new Vector2( -32, 34 )
				)
		);


		GVFPath selectedSpike;

		Vector2 beforeTruss = new Vector2( -40, 59 );
		GVFPath afterTruss = new GVFPath(
				new CubicBezierCurve(
						new Vector2( -38, 59 ),
						new Vector2( -15, 59 ),
						new Vector2( -8, 59 ),
						new Vector2( 34, 59 )
				)
		);

		Pose2D inBetweenTruss = new Pose2D( -12, 59, 0 );

		GVFPath rightBackdrop = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 24, 57 ),
						new Vector2( 48, 52 ),
						new Vector2( 30, 30 ),
						new Vector2( 49.5, 28 )
				)
		);

		GVFPath middleBackdrop = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 24, 57 ),
						new Vector2( 44, 52 ),
						new Vector2( 30, 40 ),
						new Vector2( 49.5, 36 )
				)
		);

		GVFPath leftBackdrop = new GVFPath(
				new CubicBezierCurve(
						new Vector2( 24, 57 ),
						new Vector2( 44, 52 ),
						new Vector2( 30, 40 ),
						new Vector2( 49.5, 42 )
				)
		);

		GVFPath selectedBackDrop;

//		Pose2D park = new Pose2D( 44, 12, 0 ); // right
		Pose2D park = new Pose2D( 44, 60, 0 ); // left

		while( opModeInInit( ) && !opModeIsActive( ) ) {
			robot.drive.drive( 0.03, 0, 0 );
			telemetry.addData( "position", PropProcessor.getPropPosition() );
			displayTelemetry();
			robot.update();
		}

		waitForStart();

		robot.propStream.stopStreaming();
		boolean isLeft;

		switch( PropProcessor.getPropPosition() ) {
			case MIDDLE:
				selectedBackDrop = middleBackdrop;
				selectedSpike = middleSpike;
				isLeft = false;
				break;
			case RIGHT:
				selectedBackDrop = rightBackdrop;
				selectedSpike = rightSpike;
				isLeft = false;
				break;
			case LEFT:
			case NOT_FOUND:
			default:
				selectedBackDrop = leftBackdrop;
				selectedSpike = leftSpike;
				isLeft = true;
				break;
		}

		autoMachine = new StateMachineBuilder()
				.state( AutoStates.DRIVE_TO_SPIKE )
				.onEnter( () -> robot.followPath( selectedSpike, isLeft ? 0 : 270 ) )
				.onExit( () -> {
//					robot.intake.setDeploymentState( Intake.DeploymentState.TOP_TWO );
//					robot.intake.setSpinState( Intake.SpinState.SPIKE_SCORE );
					robot.deposit.setReleaseState( Deposit.ReleaseStates.DROP_ONE );
				} )
				.transition( () -> robot.distanceToTarget() < 0.5 && isLeft, AutoStates.LEFT_INTERMEDIATE_POINT )
				.transition( () -> robot.distanceToTarget() < 0.5, AutoStates.SCORE_SPIKE  )
				.transitionTimed( 3 )

				.state( AutoStates.DETERMINE_RIGHT )
				.transition( () -> isLeft, AutoStates.LEFT_INTERMEDIATE_POINT )
				.transition( () -> !isLeft, AutoStates.SCORE_SPIKE  )

				.state( AutoStates.LEFT_INTERMEDIATE_POINT )
				.onExit( () -> robot.goToPoint( new Pose2D( -40, 32, 0 ) ) )
				.transitionTimed( 1 )

				.state( AutoStates.SCORE_SPIKE )
				.transitionTimed( 0.75 )

				.state( AutoStates.DRIVE_BEFORE_TRUSS )
				.onEnter( () -> robot.goToPoint( beforeTruss ) )
				.loop( () -> {
					if (robot.getPose().getY() > 48) robot.setTargetHeading( 0 );
				} )
				.transition( () -> robot.distanceToTarget() < 1 )

				.state( AutoStates.DRIVE_UNDER_TRUSS )
				.onEnter( () -> robot.followPath( afterTruss, 0 ) )
				.transition( () -> robot.distanceToTarget() < 1 )
//				.transition( () -> robot.getVelocity() < 1 && robot.getPose().getX() < -24, AutoStates.TRUSS_FALLBACK1 )
//				.transition( () -> robot.getVelocity() < 1 && robot.getPose().getX() < 0, AutoStates.TRUSS_FALLBACK2 )

				.state( AutoStates.DRIVE_TO_BACKDROP )
				.onEnter( () -> {
					robot.followPath( selectedBackDrop, 0 );
					robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
					robot.lift.setTarget( 125 );
				})
				.transition( () -> robot.distanceToTarget() < 1 )

				.waitState(0.5)

				.state( AutoStates.SCORE_ON_BACKDROP )
				.onEnter( () -> robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED ) )
				.transitionTimed( 0.75 )

				.state( AutoStates.BACKUP )
				.onEnter( () -> robot.goToPoint( new Vector2( robot.tracker.getPose2D().getX() - 2, robot.tracker.getPose2D().getY() ) ) )
				.transitionTimed( 1 )

				.state( AutoStates.PARK )
				.onEnter( () -> robot.goToPoint( park ) )
				.transitionTimed( 3 )

				.state( AutoStates.RESET_DEPOSIT )
				.onEnter( () -> robot.deposit.setAngleState( Deposit.AngleStates.GRAB ) )
				.transitionTimed( 1 )

				.state( AutoStates.RESET_LIFT )
				.onEnter( () -> robot.lift.setTarget( 0 ) )
				.transitionTimed( 2 )

				.state( AutoStates.SHUTDOWN )
				.onEnter( () -> requestOpModeStop() )

				.state( AutoStates.TRUSS_FALLBACK1 )
				.onEnter( () -> robot.goToPoint( beforeTruss ) )
				.transitionTimed( 2, AutoStates.DRIVE_UNDER_TRUSS )

				.state( AutoStates.TRUSS_FALLBACK2 )
				.onEnter( () -> robot.goToPoint( inBetweenTruss ) )
				.transitionTimed( 2, AutoStates.DRIVE_UNDER_TRUSS )

				.build();

		autoMachine.start();

		while( opModeIsActive() ) {
			robot.update();
			robot.lift.updatePID();
			autoMachine.update();
			displayTelemetry();
		}

	}

	public void displayTelemetry() {
		TelemetryPacket packet = new TelemetryPacket();
		Canvas field = packet.fieldOverlay( )
				.drawImage( "/dash/centerstage.webp", 0, 0, 144, 144, Math.toRadians( 180 ), 72, 72, false )
				.setAlpha( 1.0 )
				.drawGrid( 0, 0, 144, 144, 7, 7 )
				.setRotation( Math.toRadians( 270 ) );

		Pose2D pose = robot.getPose();

		double x = pose.getX();
		double y = pose.getY();
		double heading = pose.getTheta().getRadians();

		int robotRadius = 8;
		field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
		double arrowX = new Rotation2d(heading).getCos() * robotRadius, arrowY = new Rotation2d(heading).getSin() * robotRadius;
		double x1 = x, y1 = y;
		double x2 = x + arrowX, y2 = y + arrowY;
		field.strokeLine(x1, y1, x2, y2);


		switch( robot.getDriveControlState() ) {
			case GVF:
				GVFPath path = robot.getCurrentPath();
				if (path != null) {
					Vector2 firstPoint = path.getCurve( ).getP0( );

					for( double i = 0; i <= 1; i += 0.1 ) {
						Vector2 secondPoint = path.getCurve( ).calculate( i );
						field.strokeLine( firstPoint.getX( ), firstPoint.getY( ), secondPoint.getX( ), secondPoint.getY( ) );
						firstPoint = secondPoint;
					}
				}
				break;
			case P2P:
				Pose2D targetPoint = robot.getCurrentPoseTarget();
				if (targetPoint != null) field.strokeLine( robot.getPose().getX( ), robot.getPose().getY( ), targetPoint.getX( ), targetPoint.getY( ) );
				break;
		}

		FtcDashboard.getInstance().sendTelemetryPacket(packet);

		if (autoMachine != null) telemetry.addData( "state", autoMachine.getState() );
		telemetry.addData( "dist from target", robot.distanceToTarget() );
		telemetry.addData( "velocity", robot.getVelocity() );
		telemetry.update();
	}
}
