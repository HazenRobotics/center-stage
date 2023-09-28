package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SampleSwerveDrive extends SwerveDrive {

	public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients( 0, 0, 0 );
	public static PIDCoefficients HEADING_PID = new PIDCoefficients( 0, 0, 0 );

	public static double VX_WEIGHT = 1;
	public static double VY_WEIGHT = 1;
	public static double OMEGA_WEIGHT = 1;

	private TrajectorySequenceRunner trajectorySequenceRunner;

	private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint( MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH );
	private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint( MAX_ACCEL );

	private TrajectoryFollower follower;

	private AxonSwervePod frontLeft, backLeft, frontRight, backRight
	private List<AxonSwervePod> pods;

	private IMU imu;
	private VoltageSensor batteryVoltageSensor;

	private List<Integer> lastEncPositions = new ArrayList<>( );
	private List<Integer> lastEncVels = new ArrayList<>( );

	public SampleSwerveDrive( HardwareMap hardwareMap ) {
		super( kV, kA, kStatic, TRACK_WIDTH );

		follower = follower = new HolonomicPIDVAFollower( TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
				new Pose2d( 0.5, 0.5, Math.toRadians( 5.0 ) ), 0.5 );

		LynxModuleUtil.ensureMinimumFirmwareVersion( hardwareMap );

		batteryVoltageSensor = hardwareMap.voltageSensor.iterator( ).next( );

		for( LynxModule module : hardwareMap.getAll( LynxModule.class ) ) {
			module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
		}

		imu = hardwareMap.get( IMU.class, "imu" );
		IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
				DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR ) );
		imu.initialize( parameters );

		frontLeft = new AxonSwervePod(  )


	}

	@Override
	protected double getRawExternalHeading( ) {
		return imu.getRobotYawPitchRollAngles( ).getYaw( AngleUnit.RADIANS );
	}

	@NonNull
	@Override
	public List<Double> getModuleOrientations( ) {
		return null;
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions( ) {
		lastEncPositions.clear();

		List<Double> wheelPositions = new ArrayList<>();
		for (AxonSwervePod pod : pods) {
			int position = pod.getDrivePosition();
			lastEncPositions.add(position);
			wheelPositions.add(encoderTicksToInches(position));
		}
		return wheelPositions;
	}

	@Override
	public void setModuleOrientations( double v, double v1, double v2, double v3 ) {

	}

	@Override
	public void setMotorPowers( double v, double v1, double v2, double v3 ) {
		frontLeft.setDrivePower( v );
		backLeft.setDrivePower( v1 );
		frontRight.setDrivePower( v2 );
		backRight.setDrivePower( v3 );
	}

	public static TrajectoryVelocityConstraint getVelocityConstraint( double maxVel,
																	  double maxAngularVel, double trackWidth ) {
		return new MinVelocityConstraint( Arrays.asList(
				new AngularVelocityConstraint( maxAngularVel ),
				new MecanumVelocityConstraint( maxVel, trackWidth )
		) );
	}

	public static TrajectoryAccelerationConstraint getAccelerationConstraint( double maxAccel ) {
		return new ProfileAccelerationConstraint( maxAccel );
	}
}
