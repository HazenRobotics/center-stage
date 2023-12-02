package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;
import static org.firstinspires.ftc.teamcode.utils.SwervePDController.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.HeadingPDController;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

import java.util.List;

@TeleOp
@Config
public class BaseSwerve extends LinearOpMode {
	KhepriBot robot;
	GamepadEvents controller1;
	PIDController headingController;
	double drive, strafe, rotate, loop, prevTime, heading, error, adjustedHeading;
	public static double target, startHeading = Math.toRadians( 90 );
	Pose2D poseEstimate;
	boolean headingLock = false;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		Pose2D startPose = new Pose2D(0, 0 , startHeading);
		robot.setupTracker( startPose );
		target = startHeading;

		controller1 = new GamepadEvents( gamepad1 );

		headingController = new PIDController( 1, 0, 0.1 );

		waitForStart( );

		while( opModeIsActive( ) ) {
			poseEstimate = robot.getPose();
			heading = poseEstimate.getTheta().getRadians();

			if( controller1.x.onPress( ) ) {
				headingLock = !headingLock;
				if( headingLock )
					target = heading;
			}

			drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast( ) : KhepriBot.DriveSpeeds.DRIVE.getNorm( ));
			strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast( ) : KhepriBot.DriveSpeeds.STRAFE.getNorm( ));
			rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast( ) : KhepriBot.DriveSpeeds.ROTATE.getNorm( ));

			if( headingLock ) {
				error = findShortestAngularTravel( target, heading );
				rotate += headingController.calculate( error, 0 );
			}

			robot.drive.fieldCentricDrive( drive, strafe, rotate, heading - (Math.PI / 2));

			robot.tracker.updatePose();
			displayTelemetry();
			controller1.update( );
			robot.clearBulkCache();
		}
	}

	public void displayTelemetry() {
		loop = System.currentTimeMillis( );
		telemetry.addData( "hz ", 1000 / (loop - prevTime) );
		prevTime = loop;

		telemetry.addData( "heading lock", headingLock );
		telemetry.addData("where the localizer thinks it is", poseEstimate.subtract( 0, 0, new AngleRadians(Math.toRadians( 90 )) ));
		telemetry.addData( "where i want the localizer to think it is", poseEstimate);

//		x = poseEstimate.getX();
//		y = poseEstimate.getY();

//		TelemetryPacket packet = new TelemetryPacket();
//		field = packet.fieldOverlay();
//		int robotRadius = 8;
//		field.strokeCircle(x, y, robotRadius);
//		double arrowX = new Rotation2d(poseHeading).getCos() * robotRadius, arrowY = new Rotation2d(poseHeading).getSin() * robotRadius;
//		double x1 = x, y1 = y;
//		double x2 = x + arrowX, y2 = y + arrowY;
//		field.strokeLine(x1, y1, x2, y2);
//		FtcDashboard.getInstance().sendTelemetryPacket(packet);

		telemetry.update( );
	}
}
