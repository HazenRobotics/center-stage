package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scarabtuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

@TeleOp
public class TrackWidthTuning extends LinearOpMode {
	KhepriBot robot;
	AngleDegrees currentHeading, lastHeading;
	double totalHeading;

	double drive, strafe, rotate;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D(0, 0 , Math.toRadians( 90 )) );

		currentHeading = robot.tracker.getPose2D().getTheta().toAngleDegrees();
		lastHeading = currentHeading;

		waitForStart();

		while(opModeIsActive()) {
			drive = -gamepad1.left_stick_y * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.DRIVE.getFast( ) : KhepriBot.DriveSpeeds.DRIVE.getNorm( ));
			strafe = gamepad1.left_stick_x * (gamepad1.left_stick_button ? KhepriBot.DriveSpeeds.STRAFE.getFast( ) : KhepriBot.DriveSpeeds.STRAFE.getNorm( ));
			rotate = gamepad1.right_stick_x * (gamepad1.right_stick_button ? KhepriBot.DriveSpeeds.ROTATE.getFast( ) : KhepriBot.DriveSpeeds.ROTATE.getNorm( ));

			robot.drive.drive( drive, strafe, rotate );

			currentHeading = robot.tracker.getPose2D().getTheta().toAngleDegrees();
			totalHeading += currentHeading.findShortestDistance( lastHeading );

			telemetry.addData( "curr heading", currentHeading.getTheta() );
			telemetry.addData( "total heading", totalHeading );
			telemetry.update();
			robot.clearBulkCache();
			robot.tracker.updatePose();
			lastHeading = currentHeading;
		}


	}
}
