package org.firstinspires.ftc.teamcode.auto.statefactory;

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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;

@Autonomous
public class FarRedFactory extends LinearOpMode {

	KhepriBot robot;
	StateMachine autoMachine;

	public enum AutoStates {
		DRIVE_TO_SPIKE,
		SCORE_SPIKE,
		DRIVE_BEFORE_TRUSS,
		DRIVE_UNDER_TRUSS,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP,
		PARK
	}


	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( -38, -61.125, new AngleDegrees( 90 ) ) );
		robot.setupPropProcessor( PropProcessor.PropColor.RED_FAR );
		robot.setMaxAutoDriveSpeed( 0.5 );

		robot.deposit.setReleaseState( Deposit.ReleaseStates.EXTENDED );
		robot.deposit.setAngleState( Deposit.AngleStates.STRAIGHT_DOWN );
		robot.lift.setTarget( 0 );

		Pose2D leftSpike = new Pose2D( -48, -44, 270 );
		Pose2D middleSpike = new Pose2D( -40, -32, 270 );
		Pose2D rightSpike = new Pose2D( -36, -32, 180 );
		Pose2D selectedSpike;

		Pose2D beforeTruss = new Pose2D( -40, -58, 0 );
		Pose2D afterTruss = new Pose2D( 36, -57, 0 );

		Pose2D leftBackDrop = new Pose2D( 48, -28, 0 );
		Pose2D middleBackDrop = new Pose2D( 48, -34, 0 );
		Pose2D rightBackDrop = new Pose2D( 48, -42, 0 );
		Pose2D selectedBackDrop;

		Pose2D park = new Pose2D( 44, -60, 0 );

		while( opModeInInit( ) && !opModeIsActive( ) ) {
			robot.drive.drive( 0.03, 0, 0 );
			telemetry.addData( "position", PropProcessor.getPropPosition() );
			displayTelemetry();
			robot.update();
		}

		waitForStart();

		robot.propStream.stopStreaming();

		switch( PropProcessor.getPropPosition() ) {
			case LEFT:
				selectedBackDrop = leftBackDrop;
				selectedSpike = leftSpike;
				break;
			case MIDDLE:
				selectedBackDrop = middleBackDrop;
				selectedSpike = middleSpike;
				break;
			case NOT_FOUND:
			case RIGHT:
			default:
				selectedBackDrop = rightBackDrop;
				selectedSpike = rightSpike;
		}

		autoMachine = new StateMachineBuilder()
				.state( AutoStates.DRIVE_TO_SPIKE )
				.onEnter( () -> robot.goToPoint( selectedSpike ) )
				.onExit( () -> {
					robot.intake.setDeploymentState( Intake.DeploymentState.TOP_TWO );
					robot.intake.setSpinState( Intake.SpinState.SPIKE_SCORE );
				} )
				.transition( () -> robot.distanceToTarget() < 0.5 )

				.state( AutoStates.SCORE_SPIKE )
				.onExit( () -> robot.intake.foldIntake())
				.transitionTimed( 0.75 )

				.state( AutoStates.DRIVE_BEFORE_TRUSS )
				.onEnter( () -> robot.goToPoint( beforeTruss ) )
				.transition( () -> robot.distanceToTarget() < 1 )

				.state( AutoStates.DRIVE_UNDER_TRUSS )
				.onEnter( () -> robot.goToPoint( afterTruss ) )
				.transition( () -> robot.distanceToTarget() < 1 )

				.state( AutoStates.DRIVE_TO_BACKDROP )
				.onEnter( () -> {
					robot.goToPoint( selectedBackDrop );
					robot.deposit.setAngleState( Deposit.AngleStates.DROP_BACKDROP );
					robot.lift.setTarget( 80 );
				})
				.transition( () -> robot.distanceToTarget() < 1 )

				.waitState(0.5)

				.state( AutoStates.SCORE_ON_BACKDROP )
				.onEnter( () -> robot.deposit.setReleaseState( Deposit.ReleaseStates.RETRACTED ) )
				.transitionTimed( 0.75 )

				.state( AutoStates.PARK )
				.onEnter( () -> robot.goToPoint( park ) )

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
		telemetry.update();
	}
}
