package org.firstinspires.ftc.teamcode.tests;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	ColorSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = hardwareMap.get( ColorSensor.class, "color" );

		waitForStart();

		while(opModeIsActive()) {
			telemetry.addData( "red", (double) color.red( ) );
			telemetry.addData( "green", (double) color.green( ) );
			telemetry.addData( "blue", (double) color.blue( ) );
			telemetry.update( );
		}
	}
}
