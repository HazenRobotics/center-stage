package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.HeadingPDController;

import java.util.List;

@TeleOp
public class FullBotTeleOp extends LinearOpMode {

	CoaxialSwerveDrive drive;
	Lift lift;
	Deposit deposit;
	Intake intake;
	Climber climber;
	Orientation orientation;
	IMU imu;
	GamepadEvents controller1;
	boolean fieldCentric = true;
	double heading, liftPos, liftPower, intakePower, loop, loopTime;
	ElapsedTime timer;

	@Override
	public void runOpMode( ) throws InterruptedException {
		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class );

		for( LynxModule hub : hubs ) hub.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		drive = new CoaxialSwerveDrive( hardwareMap );
		lift = new Lift( hardwareMap );
		deposit = new Deposit( hardwareMap, "release", "angler" );
		intake = new Intake( hardwareMap, telemetry );


		timer = new ElapsedTime( );

		imu = hardwareMap.get( IMU.class, "imu" );

		controller1 = new GamepadEvents( gamepad1 );

		imu.initialize( new IMU.Parameters( new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.RIGHT ) ) );

		imu.resetYaw( );

		waitForStart( );

		while( opModeIsActive( ) ) {

			orientation = imu.getRobotOrientation(
					AxesReference.INTRINSIC,
					AxesOrder.XYZ,
					AngleUnit.RADIANS );
			heading = orientation.thirdAngle;

			if( controller1.x.onPress( ) )
				drive.setWheelState( drive.getWheelState() == CoaxialSwerveDrive.WheelState.DRIVE ?
									CoaxialSwerveDrive.WheelState.X : CoaxialSwerveDrive.WheelState.DRIVE );

			if( fieldCentric )
				drive.fieldCentricDrive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, orientation.thirdAngle );
			else
				drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

			liftPos = lift.getMotorPosition();
			liftPower = gamepad1.right_trigger - (gamepad1.left_trigger * (0.4 + (((620 - liftPos) / 620) * 0.2)));

			lift.setPower( liftPower );

			if( controller1.a.onPress( ) )
				deposit.releaseToggle( );

			if ( liftPos < 200 )
				deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else
				deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );

			if( gamepad1.left_bumper )
				intake.adjustUp( );
			else if( gamepad1.right_bumper )
				intake.adjustDown( );

			if(controller1.left_bumper.onPress() && timer.seconds() < 0.25)
				intake.foldIntake();

			if(controller1.left_bumper.onRelease())
				timer.reset();

			if( controller1.y.onPress( ) ) intakePower = intakePower == 1 ? 0 : 1;
			else if( controller1.b.onPress( ) ) intakePower = intakePower == -1 ? 0 : -1;

			intake.setIntakeMotorPower( intakePower );

			climber.setPower( (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0) );

			telemetry.addData( "lift pos", liftPos );
			telemetry.addData( "release pos", deposit.getReleasePosition( ) );
			telemetry.addData( "angler pos", deposit.getAnglePosition( ) );
			telemetry.addData( "heading", heading );
			telemetry.addData( "STATE", drive.getWheelState( ) );
			telemetry.addData( "field centric?", fieldCentric );
			telemetry.addData( "current draw total", hubs.get( 0 ).getCurrent( CurrentUnit.AMPS ) + hubs.get( 1 ).getCurrent( CurrentUnit.AMPS ) );
			loop = System.currentTimeMillis( );
			telemetry.addData( "hz ", 1000 / (loop - loopTime) );
			loopTime = loop;
			drive.displayWheelAngles( telemetry );

			telemetry.update( );
			controller1.update( );
		}
	}
}
