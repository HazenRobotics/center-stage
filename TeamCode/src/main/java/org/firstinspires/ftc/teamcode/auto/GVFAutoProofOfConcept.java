package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;

import java.util.ArrayList;
import java.util.ListIterator;

//@Photon
@Autonomous
@Disabled
public class GVFAutoProofOfConcept extends LinearOpMode {

	KhepriBot robot;
	Pose2D poseEstimate;
	ArrayList<GVFPath> paths;

	ListIterator<GVFPath> pathIterator;
	GVFPath currentPath;
	GVFPath.PathState pathState, lastPathState;

	ElapsedTime waitTime;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupTeleOpTracker( new Pose2D( 12, 12, new AngleDegrees( 90 ) ) );

		paths = new ArrayList<>( );
		waitTime = new ElapsedTime( );

		paths.add( new GVFPath( new CubicBezierCurve(
						new Vector2( 12, 12 ),
						new Vector2( 12, 60 ),
						new Vector2( 60, 12 ),
						new Vector2( 60, 60 )
				) )
		);


		paths.add( new GVFPath( paths.get( 0 ).getCurve( ).createC1ContinuousCurve(
				new Vector2( 20, 61 ),
				new Vector2( -23, 57 )
		)
		) );

		pathIterator = paths.listIterator( );
		currentPath = pathIterator.next( );


		telemetry.addLine( "finished building" );
		telemetry.update( );

		waitForStart( );

		while( opModeIsActive( ) ) {

			poseEstimate = robot.getPose( );

			robot.followPath( currentPath, 90 );
//			robot.goToPoint( 36, 36, 90 );
			robot.update( );

			pathState = currentPath.evaluateState( new Vector2( poseEstimate.getX(), poseEstimate.getY() ) );

			if( pathState == GVFPath.PathState.DONE ) {
				if( lastPathState != GVFPath.PathState.DONE ) waitTime.reset( );
				if( pathIterator.hasNext( ) && (currentPath.isContinuous( ) || waitTime.seconds( ) > currentPath.getWaitTime()) )
					currentPath = pathIterator.next( );
			}

			lastPathState = pathState;


			double x = poseEstimate.getX( );
			double y = poseEstimate.getY( );
			double heading = poseEstimate.getTheta( ).getRadians( );

			TelemetryPacket packet = new TelemetryPacket( );
			Canvas field = packet.fieldOverlay( )
					.drawImage( "/dash/centerstage.webp", 0, 0, 144, 144, Math.toRadians( 180 ), 72, 72, false )
					.setAlpha( 1.0 )
					.drawGrid( 0, 0, 144, 144, 7, 7 )
					.setRotation( Math.toRadians( 270 ) );

			int robotRadius = 8;
			field.strokeCircle( x, y, robotRadius );
			double arrowX = new Rotation2d( heading ).getCos( ) * robotRadius, arrowY = new Rotation2d( heading ).getSin( ) * robotRadius;
			double x1 = x, y1 = y;
			double x2 = x + arrowX, y2 = y + arrowY;
			field.strokeLine( x1, y1, x2, y2 );

			Vector2 firstPoint = currentPath.getCurve( ).getP0( );

			for( double i = 0; i <= 1; i += 0.1 ) {
				Vector2 secondPoint = currentPath.getCurve( ).calculate( i );
				field.strokeLine( firstPoint.getX( ), firstPoint.getY( ), secondPoint.getX( ), secondPoint.getY( ) );
				firstPoint = secondPoint;
			}

			packet.put( "pose", poseEstimate );
			packet.put( "path state", currentPath.evaluateState( new Vector2( x, y ) ) );

			packet.put( "FLP", robot.drive.swervePods[0].getDrivePower( ) );
			packet.put( "BLP", robot.drive.swervePods[1].getDrivePower( ) );
			packet.put( "FRP", robot.drive.swervePods[2].getDrivePower( ) );
			packet.put( "BRP", robot.drive.swervePods[3].getDrivePower( ) );
			packet.put( "isCont", currentPath.isContinuous());

			FtcDashboard.getInstance( ).sendTelemetryPacket( packet );
		}

	}
}
