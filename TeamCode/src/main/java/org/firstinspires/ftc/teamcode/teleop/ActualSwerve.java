package org.firstinspires.ftc.teamcode.teleop;

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
	GamepadEvents controller1;

	boolean fieldCentric = true;

	@Override
	public void runOpMode( ) throws InterruptedException {

		drive = new CoaxialSwerveDrive( hardwareMap );

		imu = hardwareMap.get( IMU.class, "imu" );

		controller1 = new GamepadEvents( gamepad1 );

		imu.initialize(
				new IMU.Parameters(
						new RevHubOrientationOnRobot(
								RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
								RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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

			if( controller1.a.onPress( ) ) drive.setWheelState( CoaxialSwerveDrive.WheelState.DIAMOND );
			else if ( controller1.x.onPress() ) drive.setWheelState( CoaxialSwerveDrive.WheelState.X );
			else if ( controller1.y.onPress() ) drive.setWheelState( CoaxialSwerveDrive.WheelState.DRIVE );
			else if ( controller1.b.onPress() ) fieldCentric = !fieldCentric;

			if (fieldCentric) drive.fieldCentricDrive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, -orientation.thirdAngle );
			else drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );



			telemetry.addData( "IMU X", orientation.firstAngle );
			telemetry.addData( "IMU Y", orientation.secondAngle );
			telemetry.addData( "IMU Z", orientation.thirdAngle );
			telemetry.addData( "STATE", drive.getWheelState() );
			telemetry.addData( "field centric?", fieldCentric );
			drive.displayWheelAngles( telemetry );

			telemetry.update( );
			controller1.update( );
		}
	}
}
