package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	ColorSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = hardwareMap.get( ColorSensor.class, "color" );

		waitForStart();

		while(opModeIsActive()) {
			telemetry.addData( "red", Range.clip( color.red( ) / 32, 0, 255 ) );
			telemetry.addData( "green", Range.clip( color.green( ) / 32, 0, 255 ) );
			telemetry.addData( "blue", Range.clip( color.blue( ) / 32, 0, 255 ) );
			telemetry.update( );
		}
	}
}
