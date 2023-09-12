package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;

@TeleOp
public class SpinnySwerve extends LinearOpMode {

	CoaxialSwerveDrive drive;
	double joyX, joyY, joyMag;

	@Override
	public void runOpMode( ) throws InterruptedException {

		drive = new CoaxialSwerveDrive( hardwareMap );

		waitForStart( );

		while( opModeIsActive( ) ) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );

			drive.spinny( joyMag );
		}
	}
}
