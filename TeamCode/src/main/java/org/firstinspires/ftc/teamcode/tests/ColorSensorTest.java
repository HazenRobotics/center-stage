package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeColourSensor;

@TeleOp
@Disabled
public class ColorSensorTest extends LinearOpMode {
	IntakeColourSensor color;

	@Override
	public void runOpMode( ) throws InterruptedException {
		color = new IntakeColourSensor(hardwareMap, telemetry, "color");


		waitForStart();

		while(opModeIsActive()) {
			color.addTelemetry();
			telemetry.update( );
		}
	}
}