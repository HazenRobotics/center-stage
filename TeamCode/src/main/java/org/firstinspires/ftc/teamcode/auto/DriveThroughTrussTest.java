package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.GVF.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;

@Autonomous
//@Photon
public class DriveThroughTrussTest extends LinearOpMode {

	KhepriBot robot;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( -38.5, -63.5, new AngleDegrees( 0 ) ) );

		GVFPath path = new GVFPath( new CubicBezierCurve(
				new Vector2( -38.5, -63.5 ),
				new Vector2( -53.4, -56.5 ),
				new Vector2( 6.8, -61.3 ),
				new Vector2( 18.7, -60 )

		) );

		Pose2D toBackdrop = new Pose2D( 48, -42.2, 0 );

		waitForStart();

		while(opModeIsActive()) {
			if( robot.getPose( ).getX( ) < 15 ) robot.followPath( path, 0 );
			else robot.goToPoint( toBackdrop );

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

			GVFPath currentPath = robot.getCurrentPath();
			if (currentPath != null) {
				Vector2 firstPoint = path.getCurve( ).getP0( );

				for( double i = 0; i <= 1; i += 0.1 ) {
					Vector2 secondPoint = path.getCurve( ).calculate( i );
					field.strokeLine( firstPoint.getX( ), firstPoint.getY( ), secondPoint.getX( ), secondPoint.getY( ) );
					firstPoint = secondPoint;
				}
			}
			robot.update( );
		}
	}
}
