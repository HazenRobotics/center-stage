package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.roadrunner.KhepriSwerveTwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SlingshotLauncher;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.EncoderTicksConverter;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.Units;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.IMU_EX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.ThreeWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.TwoWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.WheeledTrackerConstants;

import java.util.List;

public class KhepriBot {
	HardwareMap hw;
	Telemetry telemetry;

	public CoaxialSwerveDrive drive;
	public ThreeWheelTracker tracker;
	public Lift lift;
	public Deposit deposit;
	public Intake intake;
	public Climber climber;
	public SlingshotLauncher launcher;
	public IMU_EX imu;
	public PIDController headingController;
	public List<LynxModule> hubs;

	public enum DriveSpeeds {
		DRIVE(0.7, 1), STRAFE(0.7, 1), ROTATE(0.5, 1);
		double norm, fast;

		DriveSpeeds( double norm, double fast ) {
			this.norm = norm;
			this.fast = fast;
		}
		public double getFast( ) {
			return fast;
		}
		public double getNorm( ) {
			return norm;
		}
	}

	public KhepriBot ( HardwareMap hw, Telemetry t) {
		this.hw = hw;
		telemetry = new MultipleTelemetry( t, FtcDashboard.getInstance( ).getTelemetry( ) );

		hubs = hw.getAll( LynxModule.class );
		for( LynxModule hub : hubs ) hub.setBulkCachingMode( LynxModule.BulkCachingMode.MANUAL );

		drive = new CoaxialSwerveDrive( hw );
		lift = new Lift( hw );
		deposit = new Deposit( hw );
		intake = new Intake( hw, t );
		intake.foldIntake();
		climber = new Climber( hw );
		launcher = new SlingshotLauncher( hw );

		setupIMU( RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD );
		imu.resetYaw();

		headingController = new PIDController( 1, 0, 0.1 );
	}


	public void setupIMU( RevHubOrientationOnRobot.LogoFacingDirection logoDir, RevHubOrientationOnRobot.UsbFacingDirection usbDir ) {
		imu = new IMU_EX(hw.get(IMU.class, "imu"), AngleUnit.RADIANS);
		imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						logoDir,
						usbDir
				)
		));
	}

	public void setupTracker( Pose2D startPose ) {
		tracker = new ThreeWheelTracker(
				new Pose2D( startPose.getX(),
							startPose.getY(),
							startPose.getTheta( ).add( new AngleDegrees( 90 ) )
				),
				new WheeledTrackerConstants.ThreeWheeledTrackerConstants(
						new Vector2D( ),
						1,
						1,
						new EncoderTicksConverter(1769.9, Units.INCH),
						new EncoderTicksConverter(1768.22592593, Units.INCH),
						new EncoderTicksConverter( 1742.38148148, Units.INCH ),
						283.756949439),
				new Encoder( hw.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BRM/paraREnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BLM/perpEnc" ) ).setDirection( Encoder.Direction.FORWARD )
//				imu
		);
	}

	public void clearBulkCache( ) {
		for( LynxModule hub : hubs ) hub.clearBulkCache();
	}

	public double getRobotCurrentAmps() {
		return hubs.get( 0 ).getCurrent( CurrentUnit.AMPS ) + hubs.get( 1 ).getCurrent( CurrentUnit.AMPS );
	}
}
