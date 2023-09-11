package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.AxonAbsolutePositionEncoder;

@Config
@TeleOp(group = "Test")
public class PIDSwerveTuning extends LinearOpMode {

	CRServo crServo;
	AxonAbsolutePositionEncoder encoder;

	SwervePIDController controller;

	public static double p = 0, i = 0, d = 0;

	public static double angle = 0;

	double loopTime;

	@Override
	public void runOpMode( ) throws InterruptedException {
		crServo = hardwareMap.get( CRServo.class, "servo" );
		encoder = new AxonAbsolutePositionEncoder( hardwareMap, "encoder" );

		controller = new SwervePIDController();

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while( opModeIsActive() ) {

			if(gamepad1.a) {
				encoder.setOffset( 0 );
				encoder.setOffset( encoder.getAngle() );
			}

			controller.setPID( p, i, d );


			crServo.setPower( -controller.update( Math.toRadians( angle ), encoder.getAngle() ));

			updateTelemetry( );
		}

	}

	public void updateTelemetry( ) {
		telemetry.addData( "rotate pos RAD", encoder.getAngle( ) );
		telemetry.addData( "target angle RAD", Math.toRadians( angle ) );
		telemetry.addData( "error RAD", controller.getError() );
		telemetry.addData( "rotate pos DEG", Math.toDegrees( encoder.getAngle( ) ) );
		telemetry.addData( "target angle DEG", angle );
		telemetry.addData( "error DEG", Math.toDegrees( controller.getError( ) ) );
		telemetry.addData( "offset", encoder.getOffset() );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
