package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SlingshotLauncher;

@TeleOp
public class AirplaneSlingshotTest extends LinearOpMode {

	SlingshotLauncher launcher;
	@Override
	public void runOpMode( ) throws InterruptedException {

		launcher = new SlingshotLauncher( hardwareMap );

		waitForStart();

		while( opModeIsActive() ) {
			if (gamepad1.a) launcher.release();
			else if( gamepad1.b ) launcher.prime();
		}
	}
}
