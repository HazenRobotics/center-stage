package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.KhepriSwerveDrive;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;

@Autonomous
public class RedNearSidePark extends LinearOpMode {

	KhepriBot robot;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );

		robot.roadrunnerDrive.getLocalizer().setPoseEstimate( new Pose2d(10,-61.5, Math.toRadians( 90 ) ));

		waitForStart();

		robot.roadrunnerDrive.followTrajectorySequence( robot.roadrunnerDrive.trajectorySequenceBuilder(new Pose2d(10,-61.5, Math.toRadians( 90 )))
				.lineTo(new Vector2d( 59, -61.5 ) )
				.build());
	}
}
