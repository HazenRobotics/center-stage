package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeColourSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
	IntakeColourSensor colour;

	@Override
	public void runOpMode( ) throws InterruptedException {
		colour = new IntakeColourSensor(hardwareMap, telemetry, "color");


		waitForStart();

		while(opModeIsActive()) {
			colour.getTelemetry();
			telemetry.update( );
		}
	}
}