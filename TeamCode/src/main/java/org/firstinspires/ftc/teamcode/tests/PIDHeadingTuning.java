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

import java.util.List;

@Config
@TeleOp
public class PIDHeadingTuning extends LinearOpMode {

	CoaxialSwerveDrive drive;
	PIDController controller;
	IMU imu;

	public static double p, i, d, target;
	double power, heading, error, loopTime;
	Orientation orientation;

	@Override
	public void runOpMode( ) throws InterruptedException {
		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class);

		for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

		drive = new CoaxialSwerveDrive( hardwareMap );
		imu = hardwareMap.get( IMU.class, "imu" );
		imu.initialize( new IMU.Parameters( new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD ) ) );
		imu.resetYaw( );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		controller = new PIDController( p, i, d );

		waitForStart( );

		while( opModeIsActive( ) ) {
			orientation = imu.getRobotOrientation(
					AxesReference.INTRINSIC,
					AxesOrder.XYZ,
					AngleUnit.RADIANS );

			heading = orientation.thirdAngle;
			controller.setPID( p, i, d );

			error = findShortestAngularTravel( Math.toRadians( target ), heading );

			power = controller.calculate( error, 0 );

			drive.drive( 0, 0, power );

			updateTelemetry( );
		}
	}

	public void updateTelemetry( ) {
		telemetry.addData( "heading RAD", heading );
		telemetry.addData( "heading DEG", Math.toDegrees( heading ) );
		telemetry.addData( "target RAD", Math.toRadians( target ) );
		telemetry.addData( "target DEG", target );
		telemetry.addData( "error RAD", error );
		telemetry.addData( "error DEG", Math.toDegrees( error ) );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
