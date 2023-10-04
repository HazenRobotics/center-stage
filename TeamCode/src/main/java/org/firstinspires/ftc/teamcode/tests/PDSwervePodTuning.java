package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive.encoderOffsets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

import java.util.List;

@Config
@TeleOp(group = "Test")
public class PDSwervePodTuning extends LinearOpMode {
	AxonSwervePod pod1, pod2, pod3, pod4;

	AxonSwervePod[] pods;

	public static double p = 0, d = 0;

	public static double angle = 0;

	double loopTime;

	@Override
	public void runOpMode( ) throws InterruptedException {

		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class);
		for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

		pod1 = new AxonSwervePod( hardwareMap, "FLM/perp", false, "FLS", false,
				"FLE", encoderOffsets[0], 3.3, new double[]{0,0}, 0);
		pod2 = new AxonSwervePod( hardwareMap, "BLM", false, "BLS", false,
				"BLE", encoderOffsets[1], 3.3, new double[]{0,0}, 0);
		pod3 = new AxonSwervePod( hardwareMap, "FRM", false, "FRS", false,
				"FRE", encoderOffsets[2], 3.3, new double[]{0,0}, 0);
		pod4 = new AxonSwervePod( hardwareMap, "BRM/para", false, "BRS", false,
				"BRE", encoderOffsets[3], 3.3, new double[]{0,0}, 0);

		pods = new AxonSwervePod[]{pod1, pod2, pod3, pod4};

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while( opModeIsActive() ) {

			for( int i = 0; i < pods.length; i++ ) {
				pods[i].setPID( p,d );
				pods[i].setAngleTarget( Math.toRadians( angle ) );
				pods[i].update( 0 );
			}

			updateTelemetry( );
		}

	}

	public void updateTelemetry( ) {
		for (int i = 0; i < 4; i++) {
			telemetry.addData( "rotate pos RAD " + i, pods[i].getAngle( ) );
			telemetry.addData( "error RAD " + i, pods[i].getPIDError() );
			telemetry.addData( "rotate pos DEG " + i, Math.toDegrees( pods[i].getAngle( ) ) );
			telemetry.addData( "error DEG " + i, Math.toDegrees( pods[i].getPIDError() ) );
			telemetry.addLine();
		}
		telemetry.addLine();

		telemetry.addData( "target angle RAD", Math.toRadians( angle ) );
		telemetry.addData( "target angle DEG", angle );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
