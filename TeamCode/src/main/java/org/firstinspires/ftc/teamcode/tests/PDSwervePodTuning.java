package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

@Config
@TeleOp(group = "Test")
public class PDSwervePodTuning extends LinearOpMode {
	AxonSwervePod pod;

	public static double p = 0, i = 0, d = 0;

	public static double angle = 0;

	double loopTime;

	@Override
	public void runOpMode( ) throws InterruptedException {

		pod = new AxonSwervePod( hardwareMap, "FLM", false, "FLS", false,
				"FLE", 5.96, 3.3, new double[]{0,0}, 0);

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while( opModeIsActive() ) {

			pod.setPID( p, d );

			pod.setAngleTarget( Math.toRadians( angle ) );

			pod.update( 0 );

			updateTelemetry( );
		}

	}

	public void updateTelemetry( ) {
		telemetry.addData( "rotate pos RAD", pod.getAngle( ) );
		telemetry.addData( "target angle RAD", Math.toRadians( angle ) );
		telemetry.addData( "error RAD", pod.getPIDError() );
		telemetry.addData( "rotate pos DEG", Math.toDegrees( pod.getAngle( ) ) );
		telemetry.addData( "target angle DEG", angle );
		telemetry.addData( "error DEG", Math.toDegrees( pod.getPIDError( ) ) );
		telemetry.addData( "offset", pod.getOffset() );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
