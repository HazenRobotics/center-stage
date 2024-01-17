package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

import java.util.List;

@TeleOp
@Disabled
public class Axon360Swerve extends LinearOpMode {

	AxonSwervePod pod;
	double joyAngle, joyMag, joyX, joyY, reverseMotor = 1, loopTime;

	List<LynxModule> hubs;

	@Override
	public void runOpMode( ) throws InterruptedException {
		hubs = hardwareMap.getAll(LynxModule.class);

		for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

		pod = new AxonSwervePod( hardwareMap, "FLM/perp", "FLS", "FLE" );
		pod.reverseServo();

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while( opModeIsActive() && !isStopRequested() ) {

			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );
			if( joyMag > 0.1 ) joyAngle = -Math.atan2( joyY, joyX );

			pod.setDrivePower( joyMag * reverseMotor );
			pod.setRotatePower( gamepad1.right_stick_x );

			updateTelemetry();
		}

	}

	public void updateTelemetry() {
		telemetry.addData( "pod velocity", pod.getDriveVelo() );

		telemetry.addData( "pod angle (rad)", pod.getAngle() );
		telemetry.addData( "pod angle (deg)", Math.toDegrees( pod.getAngle() ) );
		telemetry.addData( "amps", hubs.get(0).getCurrent( CurrentUnit.AMPS ) );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update();
	}
}
