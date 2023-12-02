package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
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
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.Angle;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.IMU_EX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.InsistentThreeWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.ThreeWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.TwoWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.WheeledTrackerConstants;

import java.util.List;

public class KhepriBot {
	HardwareMap hw;
	Telemetry telemetry;
	TelemetryPacket packet;
	Canvas field;

	public CoaxialSwerveDrive drive;
	public ThreeWheelTracker tracker;
	public Lift lift;
	public Deposit deposit;
	public Intake intake;
	public Climber climber;
	public SlingshotLauncher launcher;
	public IMU_EX imu;
	public PIDController XController;
	public PIDController YController;
	public PIDController headingController;
	public List<LynxModule> hubs;
	Pose2D poseEstimate;

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

		packet = new TelemetryPacket();
		field = packet.fieldOverlay();

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

		XController = new PIDController( 0.2, 0 , 0.03 );
		YController = new PIDController( 0.2, 0 , 0.03 );
		headingController = new PIDController( 5, 0, 0.4 );
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
		tracker = new InsistentThreeWheelTracker(
				startPose.subtract( 0, 0, new AngleDegrees( 90 ) ),
				new WheeledTrackerConstants.ThreeWheeledTrackerConstants(
						new Vector2D( 1.2745623, -0.2857706 ),
						0.996350989966,
						0.998798735882,
						new EncoderTicksConverter(1769.9, Units.MILLIMETER),
						new EncoderTicksConverter(1768.22592593, Units.MILLIMETER),
						new EncoderTicksConverter( 1742.38148148, Units.MILLIMETER ),
						11.144846156),
				new Encoder( hw.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BRM/paraREnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BLM/perpEnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				imu,
				20

		);

		updateTracker();
	}

	public Pose2D getPose() {
		return poseEstimate;
	}

	public void updateTracker () {
		tracker.updatePose();
		poseEstimate = tracker.getPose2D().add( 0, 0, new AngleDegrees( 90 ) );

		int robotRadius = 8;
		double fy = poseEstimate.getY();
		double fx = poseEstimate.getX();
		field.strokeCircle(fx, fy, robotRadius);
		double heading = poseEstimate.getTheta().getRadians();
		double arrowX = new Rotation2d(heading).getCos() * robotRadius, arrowY = new Rotation2d(heading).getSin() * robotRadius;
		double x1 = fx, y1 = fy;
		double x2 = fx + arrowX, y2 = fy+ arrowY;
		field.strokeLine(x1, y1, x2, y2);
	}

	public void goToPoint( double x, double y, double heading, double travelMultiplier, double rotationMultiplier ) {
		double headingError = findShortestAngularTravel( heading, poseEstimate.getTheta().getRadians() );
		double normalizedPowerMultiplier = 12.0 / hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS );

		drive.fieldCentricDrive(
				YController.calculate(poseEstimate.getY(), y) * normalizedPowerMultiplier * (YController.getPositionError() > 2 ? travelMultiplier : 1) ,
				XController.calculate(poseEstimate.getX(), x) * normalizedPowerMultiplier * (XController.getPositionError() > 2 ? travelMultiplier : 1),
				headingController.calculate( headingError, 0 ) * normalizedPowerMultiplier * (headingError > 0.1 ? rotationMultiplier : 1),
				poseEstimate.getTheta().getRadians() - (Math.PI / 2)
		);

		field.strokeCircle(x, y, 2);
	}

	public void goToPoint( double x, double y, double heading ) {
		goToPoint( x, y, heading, 1, 1 );
	}

	public void update() {
		clearBulkCache( );
		updateTracker( );
		FtcDashboard.getInstance().sendTelemetryPacket(packet);
		telemetry.update();

		packet = new TelemetryPacket();
		field = packet.fieldOverlay();
	}

	public void clearBulkCache( ) {
		for( LynxModule hub : hubs ) hub.clearBulkCache();
	}

	public double getRobotCurrentAmps() {
		return hubs.get( 0 ).getCurrent( CurrentUnit.AMPS ) + hubs.get( 1 ).getCurrent( CurrentUnit.AMPS );
	}

	public void addTelemetryData( String label, Object data ) {
		telemetry.addData( label, data );
	}
}
