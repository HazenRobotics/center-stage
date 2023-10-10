package org.firstinspires.ftc.teamcode.tests;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	RevColorSensorV3 color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = hardwareMap.get( RevColorSensorV3.class, "color" );

		waitForStart();

		while(opModeIsActive()) {
			telemetry.addData( "red", color.red( ) );
			telemetry.addData( "green", color.green( ) );
			telemetry.addData( "blue", color.blue( ) );
			telemetry.update( );
		}
	}
}
