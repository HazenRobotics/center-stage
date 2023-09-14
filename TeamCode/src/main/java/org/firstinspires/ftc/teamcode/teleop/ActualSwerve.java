package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class ActualSwerve extends LinearOpMode {

	CoaxialSwerveDrive drive;
	Orientation orientation;
	IMU imu;
	boolean rotatePods;
	GamepadEvents controller1;

	@Override
	public void runOpMode( ) throws InterruptedException {

		drive = new CoaxialSwerveDrive( hardwareMap );

		imu = hardwareMap.get( IMU.class, "imu" );

		controller1 = new GamepadEvents( gamepad1 );

		imu.initialize(
				new IMU.Parameters(
						new RevHubOrientationOnRobot(
								RevHubOrientationOnRobot.LogoFacingDirection.UP,
								RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
						)
				)
		);

		imu.resetYaw( );

		waitForStart( );

		while( opModeIsActive( ) ) {

			orientation = imu.getRobotOrientation(
					AxesReference.INTRINSIC,
					AxesOrder.XYZ,
					AngleUnit.RADIANS
			);

			if( controller1.a.onPress( ) ) drive.toggleLock( );

			rotatePods = (Math.abs(gamepad1.left_stick_x) > 0.02) ||
					(Math.abs(-gamepad1.left_stick_y) > 0.02) ||
					(Math.abs(gamepad1.right_stick_x) > 0.02);

			drive.move( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, -orientation.thirdAngle, rotatePods );

			telemetry.addData( "IMU X", orientation.firstAngle );
			telemetry.addData( "IMU Y", orientation.secondAngle );
			telemetry.addData( "IMU Z", orientation.thirdAngle );
			telemetry.update( );
			controller1.update( );
		}
	}
}
