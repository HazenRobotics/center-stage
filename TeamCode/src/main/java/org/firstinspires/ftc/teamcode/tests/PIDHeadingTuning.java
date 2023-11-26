package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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

import java.util.List;

@Config
@TeleOp
public class PIDHeadingTuning extends LinearOpMode {

	KhepriBot robot;
	PIDController controller;

	public static double p, i, d, target;
	double power, heading, error, loop, prevTime;
	Orientation orientation;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new KhepriBot( hardwareMap, telemetry );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		controller = new PIDController( p, i, d );

		waitForStart( );

		while( opModeIsActive( ) ) {
			heading = robot.imu.getRobotYawPitchRollAngles().getYaw( AngleUnit.RADIANS );

			controller.setPID( p, i, d );

			error = findShortestAngularTravel( Math.toRadians( target ), heading );

			power = controller.calculate( error, 0 );

			robot.drive.drive( 0, 0, power );

			updateTelemetry( );
			robot.clearBulkCache();
		}
	}

	public void updateTelemetry( ) {
		telemetry.addData( "heading RAD", heading );
		telemetry.addData( "heading DEG", Math.toDegrees( heading ) );
		telemetry.addData( "target RAD", Math.toRadians( target ) );
		telemetry.addData( "target DEG", target );
		telemetry.addData( "error RAD", error );
		telemetry.addData( "error DEG", Math.toDegrees( error ) );

		loop = System.currentTimeMillis( );
		telemetry.addData( "hz ", 1000 / (loop - prevTime) );
		prevTime = loop;

		telemetry.update( );
	}
}
