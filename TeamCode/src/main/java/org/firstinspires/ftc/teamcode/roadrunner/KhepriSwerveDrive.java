package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.KhepriDriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive.encoderOffsets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;
import org.firstinspires.ftc.teamcode.roadrunner.kinematics.SwerveDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class KhepriSwerveDrive extends SwerveDrive {

	public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients( 8, 0, 0 );
	public static PIDCoefficients HEADING_PID = new PIDCoefficients( 8, 0, 0 );

	public static double VX_WEIGHT = 1;
	public static double VY_WEIGHT = 1;
	public static double OMEGA_WEIGHT = 1;

	private TrajectorySequenceRunner trajectorySequenceRunner;

	private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint( MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH );
	private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint( MAX_ACCEL );

	private TrajectoryFollower follower;

	private AxonSwervePod frontLeft, backLeft, frontRight, backRight;
	private List<AxonSwervePod> pods;

	private IMU imu;
	private VoltageSensor batteryVoltageSensor;

	private List<Integer> lastEncPositions = new ArrayList<>( );
	private List<Integer> lastEncVels = new ArrayList<>( );

	public KhepriSwerveDrive( HardwareMap hardwareMap ) {
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
				KhepriDriveConstants.LOGO_FACING_DIR, KhepriDriveConstants.USB_FACING_DIR ) );
		imu.initialize( parameters );

		frontLeft = new AxonSwervePod( hardwareMap,"FLM/paraEnc", false, "FLS", false,
				"FLE", encoderOffsets[0], 3.3, new double[]{ 0.6, 0.0065 }, 28 * 8 );
		backLeft = new AxonSwervePod( hardwareMap,"BLM/climbEnc", false, "BLS", false,
				"BLE", encoderOffsets[1], 3.3, new double[]{ 0.6, 0.0065 }, 28 * 8 );
		frontRight = new AxonSwervePod( hardwareMap,"FRM", false, "FRS", false,
				"FRE", encoderOffsets[2], 3.3, new double[]{ 0.6, 0.0065 }, 28 * 8 );
		backRight = new AxonSwervePod( hardwareMap,"BRM/perpEnc", false, "BRS", false,
				"BRE", encoderOffsets[3], 3.3, new double[]{ 0.6, 0.0065 }, 28 * 8 );

		pods = Arrays.asList( frontLeft, backLeft, frontRight, backRight );

		for (AxonSwervePod pod : pods) {
			MotorConfigurationType motorConfigurationType = pod.motor.getMotorType().clone();
			motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
			pod.motor.setMotorType(motorConfigurationType);
		}

		List<Integer> lastTrackingEncPositions = new ArrayList<>();
		List<Integer> lastTrackingEncVels = new ArrayList<>();

		// TODO: if desired, use setLocalizer() to change the localization method
//		setLocalizer(new KhepriSwerveTwoDeadWheelLocalizer(hardwareMap, this));

		trajectorySequenceRunner = new TrajectorySequenceRunner(
				follower, HEADING_PID, batteryVoltageSensor,
				lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
		);
	}

	@Override
	protected double getRawExternalHeading( ) {
		return imu.getRobotYawPitchRollAngles( ).getYaw( AngleUnit.RADIANS );
	}

	@Override
	public Double getExternalHeadingVelocity() {
		return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
	}
	
	@NonNull
	@Override
	public List<Double> getModuleOrientations( ) {
		return Arrays.asList( frontLeft.getAngle( ), backLeft.getAngle( ),
				frontRight.getAngle(), backRight.getAngle() );
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
		frontLeft.setAngleTarget( v );
		backLeft.setAngleTarget( v1 );
		frontRight.setAngleTarget( v2 );
		backRight.setAngleTarget( v3 );
	}

	public void stopPodRotation() {
		frontLeft.setRotatePower( 0 );
		backLeft.setRotatePower( 0 );
		frontRight.setRotatePower( 0 );
		backRight.setRotatePower( 0 );
	}

	@Override
	public void setMotorPowers( double v, double v1, double v2, double v3 ) {
		frontLeft.update( v );
		backLeft.update( v1 );
		frontRight.update( v2 );
		backRight.update( v3 );
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

	public TrajectoryBuilder trajectoryBuilder( Pose2d startPose) {
		return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
		return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
		return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectorySequenceBuilder trajectorySequenceBuilder( Pose2d startPose) {
		return new TrajectorySequenceBuilder(
				startPose,
				VEL_CONSTRAINT, ACCEL_CONSTRAINT,
				MAX_ANG_VEL, MAX_ANG_ACCEL
		);
	}

	public void turnAsync(double angle) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder(getPoseEstimate())
						.turn(angle)
						.build()
		);
	}

	public void turn(double angle) {
		turnAsync(angle);
		waitForIdle();
	}

	public void followTrajectoryAsync( Trajectory trajectory) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder(trajectory.start())
						.addTrajectory(trajectory)
						.build()
		);
	}

	public void followTrajectory(Trajectory trajectory) {
		followTrajectoryAsync(trajectory);
		waitForIdle();
	}

	public void followTrajectorySequenceAsync( TrajectorySequence trajectorySequence) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
	}

	public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
		followTrajectorySequenceAsync(trajectorySequence);
		waitForIdle();
	}

	public Pose2d getLastError() {
		return trajectorySequenceRunner.getLastPoseError();
	}

	public void update() {
		updatePoseEstimate();
		DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
		if (signal != null) setDriveSignal(signal);
	}

	public void waitForIdle() {
		while (!Thread.currentThread().isInterrupted() && isBusy())
			update();
	}

	public boolean isBusy() {
		return trajectorySequenceRunner.isBusy();
	}

	public void setMode( DcMotor.RunMode runMode) {
		for (AxonSwervePod pod : pods) {
			pod.motor.setMode(runMode);
		}
	}
	public void setWeightedDrivePower(Pose2d drivePower) {
		Pose2d vel = drivePower;

		if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
				+ Math.abs(drivePower.getHeading()) > 1) {
			// re-normalize the powers according to the weights
			double denom = VX_WEIGHT * Math.abs(drivePower.getX())
					+ VY_WEIGHT * Math.abs(drivePower.getY())
					+ OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

			vel = new Pose2d(
					VX_WEIGHT * drivePower.getX(),
					VY_WEIGHT * drivePower.getY(),
					OMEGA_WEIGHT * drivePower.getHeading()
			).div(denom);
		}

		setDrivePower(vel);
	}
}
