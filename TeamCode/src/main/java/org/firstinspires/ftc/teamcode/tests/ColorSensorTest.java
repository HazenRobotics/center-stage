package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;

import java.util.Arrays;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	IntakeColorSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = new IntakeColorSensor(hardwareMap, telemetry, "color");


		waitForStart();

		while(opModeIsActive()) {
			color.readPixelColor();
			telemetry.addData("Colour: ", color.getPixelColor());
			telemetry.addData("HSV: ", Arrays.toString(color.getHSV()));
			telemetry.update( );
		}
	}
}