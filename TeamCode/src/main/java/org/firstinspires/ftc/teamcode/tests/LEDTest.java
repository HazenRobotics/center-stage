package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LEDTest extends LinearOpMode {
	RevBlinkinLedDriver driver;
	@Override
	public void runOpMode( ) throws InterruptedException {
		driver = hardwareMap.get( RevBlinkinLedDriver.class, "lights" );

		waitForStart();

		while (opModeIsActive()) {
			if( gamepad1.a ) driver.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLUE );
			if( gamepad1.b ) driver.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED );
			if( gamepad1.y ) driver.setPattern( RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE );
			if( gamepad1.x ) driver.setPattern( RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2 );
		}
	}
}
