package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;

@Autonomous
public class BlueNearSidePark extends LinearOpMode {

	KhepriBot robot;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );

		waitForStart();

		robot.roadrunnerDrive.followTrajectorySequence( robot.roadrunnerDrive.trajectorySequenceBuilder(new Pose2d(10,61.5, Math.toRadians( 270 )))
				.splineToConstantHeading(new Vector2d( 10, 35 ), Math.toRadians( 270 ))
				.turn( Math.toRadians( 90 ) )
//				.turn( Math.toRadians( 0 ) )
//				.turn( Math.toRadians( -90 ) )
				.waitSeconds( 1.5 )
				.splineToConstantHeading( new Vector2d( 59, 10 ), 0 )
				.build());
	}
}
