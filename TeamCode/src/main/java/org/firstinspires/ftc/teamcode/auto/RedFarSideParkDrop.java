package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

@Autonomous
public class RedFarSideParkDrop extends LinearOpMode {

	KhepriBot robot;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR_AUTO );
		robot.deposit.setReleasePosition( Deposit.ReleaseStates.EXTENDED );

		robot.roadrunnerDrive.getLocalizer().setPoseEstimate( new Pose2d(-36.5,-61.5, Math.toRadians( 90 ) ));

		waitForStart();

		robot.roadrunnerDrive.followTrajectorySequence( robot.roadrunnerDrive.trajectorySequenceBuilder(new Pose2d(-36.5,-61.5, Math.toRadians( 90 )))
				.splineToConstantHeading(new Vector2d( -35, -35 ), Math.toRadians( 90 ))
				.turn( Math.toRadians( -90 ) )
//				.turn( Math.toRadians( 0 ) )
//				.turn( Math.toRadians( -90 ) /)
				.waitSeconds( 1.5 )
				.setReversed( true )
				.splineToLinearHeading( new Pose2d( -35, -10, Math.toRadians( 0 ) ), 0 )
				.splineTo( new Vector2d( 35, -10 ), 0 )
				.splineToConstantHeading( new Vector2d( 53, -10 ), 0 )
				.addTemporalMarker( () -> robot.deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED ) )
				.waitSeconds( 1 )
				.addTemporalMarker( () -> robot.deposit.setAnglePosition( Deposit.AngleStates.GRAB ) )
				.build());
	}
}
