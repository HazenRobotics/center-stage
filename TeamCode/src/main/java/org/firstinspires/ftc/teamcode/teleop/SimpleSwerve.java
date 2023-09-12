package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;

import java.util.List;

@TeleOp
public class SimpleSwerve extends LinearOpMode {

	CoaxialSwerveDrive drive;
	IMU imu;
	Orientation orientation;
	double joyX, joyY, joyMag, joyAngle;

	@Override
	public void runOpMode( ) throws InterruptedException {

		drive = new CoaxialSwerveDrive( hardwareMap );
		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class);

		imu = hardwareMap.get( IMU.class, "imu" );

		imu.initialize(
				new IMU.Parameters(
						new RevHubOrientationOnRobot(
								RevHubOrientationOnRobot.LogoFacingDirection.UP,
								RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
						)
				)
		);

		imu.resetYaw();

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart( );

		while( opModeIsActive( ) ) {
			orientation = imu.getRobotOrientation(
					AxesReference.INTRINSIC,
					AxesOrder.XYZ,
					AngleUnit.RADIANS
			);
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );
			if( joyMag > 0.1 ) joyAngle = Math.atan2( joyY, joyX );

			if (Math.abs(gamepad1.right_stick_x) > 0.1)
				drive.spinny( gamepad1.right_stick_x );
			else
				drive.naiveDrive( joyMag, joyAngle - orientation.thirdAngle);

			telemetry.addData("amps", hubs.get(0).getCurrent( CurrentUnit.AMPS ));
			telemetry.addData( "joy angle", joyAngle );
			telemetry.addData( "IMU X", orientation.firstAngle);
			telemetry.addData( "IMU Y", orientation.secondAngle);
			telemetry.addData( "IMU Z", orientation.thirdAngle);
			telemetry.update();

		}
	}
}
