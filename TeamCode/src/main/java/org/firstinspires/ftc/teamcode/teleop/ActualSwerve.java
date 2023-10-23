package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

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
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.HeadingPDController;

import java.util.List;

@TeleOp
public class ActualSwerve extends LinearOpMode {

	CoaxialSwerveDrive drive;
	Orientation orientation;
	IMU imu;
	GamepadEvents controller1;
	HeadingPDController headingController;
	boolean fieldCentric = true;
	boolean headingLock = false;
	double heading;
	@Override
	public void runOpMode( ) throws InterruptedException {

		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class);

		for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

		drive = new CoaxialSwerveDrive( hardwareMap );

		imu = hardwareMap.get( IMU.class, "imu" );

		controller1 = new GamepadEvents( gamepad1 );

		imu.initialize( new IMU.Parameters( new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.RIGHT ) ) );

		imu.resetYaw( );

		headingController = new HeadingPDController( );
		headingController.setPD( 0.4, 0.038 );

		waitForStart( );

		while( opModeIsActive( ) ) {

			orientation = imu.getRobotOrientation(
					AxesReference.INTRINSIC,
					AxesOrder.XYZ,
					AngleUnit.RADIANS );
			heading = orientation.thirdAngle;

			if( controller1.a.onPress( ) )
				drive.setWheelState( CoaxialSwerveDrive.WheelState.DIAMOND );
			else if( controller1.x.onPress( ) )
				drive.setWheelState( CoaxialSwerveDrive.WheelState.X );
			else if( controller1.y.onPress( ) )
				drive.setWheelState( CoaxialSwerveDrive.WheelState.DRIVE );
			else if( controller1.b.onPress( ) ) {
				headingLock = true;
				headingController.setTargetHeading( heading );
			}

			if (Math.abs(gamepad1.right_stick_x) > 0)
				headingLock = false;

			if( fieldCentric ) {
				drive.fieldCentricDrive( -gamepad1.left_stick_y,
						gamepad1.left_stick_x,
						gamepad1.right_stick_x + (headingLock ? headingController.update( -heading ) : 0),
						orientation.thirdAngle );
			} else {
				drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
			}

			telemetry.addData( "heading", heading );
			telemetry.addData( "STATE", drive.getWheelState( ) );
			telemetry.addData( "field centric?", fieldCentric );
			drive.displayWheelAngles( telemetry );

			telemetry.update( );
			controller1.update( );
		}
	}
}
