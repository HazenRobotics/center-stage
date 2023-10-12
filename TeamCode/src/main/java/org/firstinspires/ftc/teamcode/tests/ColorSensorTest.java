package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	IntakeColorSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {

		color = new IntakeColorSensor(hardwareMap, "color");


		waitForStart();

		while(opModeIsActive()) {
//			telemetry.addData( "red", Range.clip( color.red( ) / 255, 0, 255 ) );
//			telemetry.addData( "green", Range.clip( color.green( ) / 255, 0, 255 ) );
//			telemetry.addData( "blue", Range.clip( color.blue( ) / 255, 0, 255 ) );
//			telemetry.update( );
		}
	}
}
