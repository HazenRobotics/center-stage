package org.firstinspires.ftc.teamcode.tests;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode {
	ColorSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = hardwareMap.get( ColorSensor.class, "color" );

		waitForStart();

		while(opModeIsActive()) {
			telemetry.addData( "red", (double) color.red( ) / 256 );
			telemetry.addData( "green", (double) color.green( ) / 256 );
			telemetry.addData( "blue", (double) color.blue( ) / 256 );
			telemetry.update( );
		}
	}
}
