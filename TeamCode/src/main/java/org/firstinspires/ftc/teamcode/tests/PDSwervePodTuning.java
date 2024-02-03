package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive.encoderOffsets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

import java.util.List;

@Config
@TeleOp(group = "Test")
//@Disabled
public class PDSwervePodTuning extends LinearOpMode {
	CoaxialSwerveDrive drive;
	AxonSwervePod pod;

	public static double p = 0, d = 0, s = 0, podNum = 0;

	public static double angle = 0;

	double loopTime;

	@Override
	public void runOpMode( ) throws InterruptedException {

		List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class);
		for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

		drive = new CoaxialSwerveDrive( hardwareMap );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();


		while( opModeIsActive() ) {

			pod = drive.swervePods[(int) podNum];

			pod.setPID( p, d );
			pod.setKs( s );
			pod.setAngleTarget( Math.toRadians( angle ) );
			pod.update( 0 );

			updateTelemetry( );
		}

	}

	public void updateTelemetry( ) {
		telemetry.addData( "rotate pos RAD ", pod.getAngle( ) );
		telemetry.addData( "error RAD ", pod.getPIDError() );
		telemetry.addData( "rotate pos DEG ", Math.toDegrees( pod.getAngle( ) ) );
		telemetry.addData( "error DEG ", Math.toDegrees( pod.getPIDError() ) );
		telemetry.addData( "target angle RAD", Math.toRadians( angle ) );
		telemetry.addData( "target angle DEG", angle );
		telemetry.addLine();

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
