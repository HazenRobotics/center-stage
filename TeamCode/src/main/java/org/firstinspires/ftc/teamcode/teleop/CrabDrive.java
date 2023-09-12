package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;

@TeleOp
public class CrabDrive extends LinearOpMode {

	CoaxialSwerveDrive drive;
	double joyX, joyY, joyMag, joyAngle;

	@Override
	public void runOpMode( ) throws InterruptedException {

		drive = new CoaxialSwerveDrive( hardwareMap );

		waitForStart( );

		while( opModeIsActive( ) ) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );
			if( joyMag > 0.1 ) joyAngle = -Math.atan2( joyY, joyX );

			drive.naiveDrive( joyMag, joyAngle );
		}
	}
}
