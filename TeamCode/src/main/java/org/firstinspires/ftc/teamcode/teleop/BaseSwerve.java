package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;
import org.opencv.core.Point3;

@TeleOp
@Config
public class BaseSwerve extends LinearOpMode {
	KhepriBot robot;
	GamepadEvents controller1;
	PIDController headingController;

	double drive, strafe, rotate, loop, prevTime, heading, error, intakeDeployAngle = 0.215;
	public static double target, startHeading = 90;
	Pose2D poseEstimate;
	boolean headingLock = false;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		Pose2D startPose = new Pose2D(0, 0 , new AngleDegrees( 90 ) );

//		aprilTagUtil = new AprilTagUtil( hardwareMap );

		robot.setupTeleOpTracker( startPose ); // sets up perfectly
//		robot.setupAutoTracker( startPose ); // offset by like 85 degrees somehow
		target = startHeading;

		controller1 = new GamepadEvents( gamepad1 );

		headingController = new PIDController( 1, 0, 0.1 );

		while( opModeInInit() ) robot.tracker.reset();

		waitForStart( );

		while( opModeIsActive( ) ) {
			poseEstimate = robot.getPose();
			heading = poseEstimate.getTheta().getRadians();

			drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast( ) : KhepriBot.DriveSpeeds.DRIVE.getNorm( ));
			strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast( ) : KhepriBot.DriveSpeeds.STRAFE.getNorm( ));
			rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast( ) : KhepriBot.DriveSpeeds.ROTATE.getNorm( )) + 0.03;

			robot.drive.drive( drive, strafe, rotate );


			displayTelemetry();
			robot.update();

//			if (robot.getPose().getX() > 30 && robot.getPose().getX() < 48) {
//				Point3 pose = aprilTagUtil.getPositionBasedOnTag();
//
//				if (!pose.equals( new Point3( 0, 0, 0 ) ))
//					robot.tracker.setPose2D( new Pose2D( pose.x-5, pose.y, robot.tracker.getPose2D().getTheta()) );
//			}

			controller1.update();
		}
	}

	public void displayTelemetry() {
		loop = System.currentTimeMillis( );
		telemetry.addData( "hz ", 1000 / (loop - prevTime) );
		prevTime = loop;


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

		packet.put( "heading lock", headingLock );
		packet.put( "pose", poseEstimate );

		FtcDashboard.getInstance( ).sendTelemetryPacket( packet );
	}
}
