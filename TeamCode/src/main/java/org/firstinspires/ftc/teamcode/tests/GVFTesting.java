package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.util.ArrayList;
import java.util.ListIterator;

@TeleOp
@Config
@Disabled
public class GVFTesting extends LinearOpMode {

	public static double robotX, robotY;
	public static double P0X = -48, P0Y = -48,
			P1X = -36, P1Y = 36,
			P2X = 36, P2Y = -36,
			P3X = 48, P3Y = 48,
			P4X = 0, P4Y = 60,
			P5X = 0, P5Y = 0;


	ArrayList<GVFPath> paths;
	GamepadEvents controller;
	GVFPath currentPath;
	GVFPath.PathState pathState, lastPathState;
	boolean followingPath = false;
	ListIterator<GVFPath> pathIterator;
	ElapsedTime waitTime;

	@Override
	public void runOpMode( ) throws InterruptedException {
		FtcDashboard dashboard = FtcDashboard.getInstance( );
		Telemetry dashboardTelemetry = dashboard.getTelemetry( );
		controller = new GamepadEvents( gamepad1 );

		paths = new ArrayList<>( );
		waitTime = new ElapsedTime( );

		paths.add( new GVFPath( new CubicBezierCurve(
						new Vector2( P0X, P0Y ),
						new Vector2( P1X, P1Y ),
						new Vector2( P2X, P2Y ),
						new Vector2( P3X, P3Y )
				),
						2
				)
		);

		paths.get( 0 ).setContinuous( false );

		paths.add( new GVFPath( paths.get( 0 ).getCurve( ).createC1ContinuousCurve(
						new Vector2( P4X, P4Y ),
						new Vector2( P5X, P5Y )
				))
		);

		pathIterator = paths.listIterator( );
		currentPath = pathIterator.next( );

		waitForStart( );

		while( opModeIsActive( ) ) {
			TelemetryPacket packet = new TelemetryPacket( );
			Canvas field = packet.fieldOverlay( )
					.drawImage( "/dash/centerstage.webp", 0, 0, 144, 144, Math.toRadians( 180 ), 72, 72, false )
					.setAlpha( 1.0 )
					.drawGrid( 0, 0, 144, 144, 7, 7 )
					.setRotation( Math.toRadians( 270 ) );


			field.setFill( "blue" )
					.fillCircle( robotX, robotY, 5 )
					.setFill( "red" )
					.fillCircle( P0X, P0Y, 1 )
					.setFill( "green" )
					.fillCircle( P1X, P1Y, 1 )
					.setFill( "orange" )
					.fillCircle( P2X, P2Y, 1 )
					.setFill( "pink" )
					.fillCircle( P3X, P3Y, 1 )
					.setFill( "purple" );

			Vector2 firstPoint = currentPath.getCurve( ).getP0( );

			for( double i = 0; i <= 1; i += 0.1 ) {
				Vector2 secondPoint = currentPath.getCurve( ).calculate( i );
				field.strokeLine( firstPoint.getX( ), firstPoint.getY( ), secondPoint.getX( ), secondPoint.getY( ) );
				firstPoint = secondPoint;
			}

			Vector2 currentPos = new Vector2( robotX, robotY );

			double xMovement = 0, yMovement = 0;

			if( pathState != GVFPath.PathState.DONE ) {
				Vector2 navVector = currentPath.calculateGuidanceVector( currentPos );
				xMovement = navVector.getX( );
				yMovement = navVector.getY( );
			}

			pathState = currentPath.evaluateState( currentPos );

			if( pathState == GVFPath.PathState.DONE ) {
				if( lastPathState != GVFPath.PathState.DONE ) waitTime.reset( );
				if( pathIterator.hasNext( ) && (currentPath.isContinuous( ) || waitTime.seconds( ) > currentPath.getWaitTime()) )
					currentPath = pathIterator.next( );
			}

			field.setFill( "yellow" )
					.strokeLine( robotX, robotY, robotX + xMovement * 20, robotY + yMovement * 20 );

			packet.put( "x power", xMovement );
			packet.put( "y power", yMovement );
			packet.put( "path state", pathState );
			packet.put( "dist to end", currentPath.getDistanceFromEnd( currentPos ) );

			if( controller.a.onPress( ) ) followingPath = !followingPath;

			if( followingPath ) {
				robotX += xMovement * 0.01;
				robotY += yMovement * 0.01;
			}

			lastPathState = pathState;
			dashboard.sendTelemetryPacket( packet );
			dashboardTelemetry.update( );
			controller.update( );
		}
	}
}
