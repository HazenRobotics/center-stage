package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
//import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RGBController;
import org.firstinspires.ftc.teamcode.subsystems.SlingshotLauncher;
import org.firstinspires.ftc.teamcode.utils.GVF.GVFPath;
import org.firstinspires.ftc.teamcode.utils.GVF.Vector2;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.EncoderTicksConverter;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.Units;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.IMU_EX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.InsistentThreeWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.ThreeWheelTracker;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker.WheeledTrackerConstants;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

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

	public RGBController rgbController;
	public IMU_EX imu;
	public PIDController XController;
	public PIDController YController;
	public PIDController autoHeadingController;
	public PIDController teleOpHeadingController;
	public List<LynxModule> hubs;
	Pose2D poseEstimate;
	public static double normalizedPowerMultiplier;
	public ElapsedTime voltagePollTimer;
//	PhotonLynxVoltageSensor photonVoltageSensor;
	public GVFPath currentPath;
	public double targetHeading;
	public Pose2D currentPoseTarget;

	public double maxAutoDriveSpeed = 0.75;

	static double loop, loopTime, currentHz, prevTime;

	public static final double robotMass = 34.1;

	public VisionPortal propStream;

	public enum DriveSpeeds {
		DRIVE(0.7, 1), STRAFE(0.7, 1), ROTATE(0.5, 1);
		final double norm, fast;

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

	public enum DriveControlState {
		GVF,
		P2P,
		MANUAL
	}

	DriveControlState driveControlState = DriveControlState.MANUAL;

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
		rgbController = new RGBController( hw );

		setupIMU( RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP );
		imu.resetYaw();

		XController = new PIDController( 0.2, 0 , 0.03 );
		YController = new PIDController( 0.2, 0 , 0.03 );
		autoHeadingController = new PIDController( 5, 0, 0.5 );
		teleOpHeadingController = new PIDController( 1, 0, 0.1 );

		voltagePollTimer = new ElapsedTime();
		normalizedPowerMultiplier = 12.0 / hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS );
//		photonVoltageSensor = hw.getAll(PhotonLynxVoltageSensor.class).iterator().next();
//		normalizedPowerMultiplier = 12.0 / photonVoltageSensor.getCachedVoltage();
	}

	public void setupPropProcessor( PropProcessor.PropColor color ) {
		propStream = new VisionPortal.Builder()
				.setCamera(hw.get( WebcamName.class, "front"))
				.addProcessor( new PropProcessor().setPropColor( color ) )
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.enableLiveView( true )
				.setAutoStopLiveView(true)
				.build();
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

	public void setupAutoTracker( Pose2D startPose ) {
		tracker = new InsistentThreeWheelTracker(
				startPose.subtract( 0, 0, new AngleDegrees( 90 ) ),
				new WheeledTrackerConstants.ThreeWheeledTrackerConstants(
						new Vector2D( -0.8912765, 0.096239 ),
						1.00036903243,
						0.999100313868,
						new EncoderTicksConverter(947.073015873, Units.MILLIMETER),
						new EncoderTicksConverter(945.244444444, Units.MILLIMETER),
						new EncoderTicksConverter( 947.382222222, Units.MILLIMETER ),
						13.7633126736),
				new Encoder( hw.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.REVERSE ),
				new Encoder( hw.get( DcMotorEx.class, "BRM/paraREnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BLM/perpEnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				imu,
				20

		);
		updateTracker();
	}

	public void setupTeleOpTracker( Pose2D startPose ) {
		tracker = new ThreeWheelTracker(
				startPose.subtract( 0, 0, new AngleDegrees( 90 ) ),
				new WheeledTrackerConstants.ThreeWheeledTrackerConstants(
						new Vector2D( -0.8912765, 0.096239 ),
						1.00036903243,
						0.999100313868,
						new EncoderTicksConverter(947.073015873, Units.MILLIMETER),
						new EncoderTicksConverter(945.244444444, Units.MILLIMETER),
						new EncoderTicksConverter( 947.382222222, Units.MILLIMETER ),
						13.7633126736),
				new Encoder( hw.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.REVERSE ),
				new Encoder( hw.get( DcMotorEx.class, "BRM/paraREnc" ) ).setDirection( Encoder.Direction.FORWARD ),
				new Encoder( hw.get( DcMotorEx.class, "BLM/perpEnc" ) ).setDirection( Encoder.Direction.FORWARD )
		);
		updateTracker();
	}

	public Pose2D getPose() {
		return poseEstimate;
	}
	public Vector2 getVector2() {
		return new Vector2( poseEstimate.getX(), poseEstimate.getY() );
	}

	public void updateTracker() {
		tracker.updatePose();
		poseEstimate = tracker.getPose2D().add( 0, 0, new AngleDegrees( 90 ) );
	}

	public void goToPoint( double x, double y, double heading ) {
		driveControlState = DriveControlState.P2P;
		currentPoseTarget = new Pose2D( x, y, heading );
	}

	public double distanceToTarget() {
		switch( driveControlState ) {
			case P2P:
				return Math.hypot( currentPoseTarget.getX() - poseEstimate.getX(),
						currentPoseTarget.getY() - poseEstimate.getY() );
			case GVF:
				return currentPath.getDistanceFromEnd( getVector2() );
			case MANUAL:
			default:
				return 0;
		}
	}
	public void goToPoint( Pose2D pose ) {
		goToPoint( pose.getX(), pose.getY(), pose.getTheta().getDegrees() );
	}
	public void followPath( GVFPath path, double targetHeading ) {
		setDriveControlState( DriveControlState.GVF );
		currentPath = path;
		this.targetHeading = targetHeading;
	}

	public GVFPath getCurrentPath( ) {
		return currentPath;
	}

	public Pose2D getCurrentPoseTarget( ) {
		return currentPoseTarget;
	}

	public void updateDrive() {
		double headingError;
		switch( driveControlState ) {
			case P2P:
				headingError = findShortestAngularTravel( Math.toRadians( currentPoseTarget.getTheta().getDegrees() ), poseEstimate.getTheta().getRadians() );

				double yPow = YController.calculate(poseEstimate.getY(), currentPoseTarget.getY());
				double xPow = XController.calculate(poseEstimate.getX(), currentPoseTarget.getX());

				if (Math.hypot( YController.getPositionError(), XController.getPositionError() ) > 2) drive.setMaxSpeed( maxAutoDriveSpeed );
				else drive.setMaxSpeed( 1 );

				drive.fieldCentricDrive(
						yPow * normalizedPowerMultiplier,
						xPow * normalizedPowerMultiplier,
						autoHeadingController.calculate( headingError, 0 ) * normalizedPowerMultiplier,
						poseEstimate.getTheta().getRadians()
				);
				break;
			case GVF:
				Vector2 currentPos = new Vector2( poseEstimate.getX( ), poseEstimate.getY( ) );
				headingError = findShortestAngularTravel( Math.toRadians( targetHeading ), poseEstimate.getTheta( ).getRadians( ) );

				if (currentPath.getDistanceFromEnd( currentPos ) > 2) drive.setMaxSpeed( maxAutoDriveSpeed );
				else drive.setMaxSpeed( 1 );

				switch( currentPath.evaluateState( currentPos ) ) {
					case FOLLOW_PATH:
						Vector2 powerVector = currentPath.calculateGuidanceVector( currentPos );
						drive.fieldCentricDrive(
								powerVector.getY( ) * normalizedPowerMultiplier,
								powerVector.getX( ) * normalizedPowerMultiplier,
								teleOpHeadingController.calculate( headingError, 0 ),
								poseEstimate.getTheta( ).getRadians( )
						);
						break;
					case USE_PID:
					case DONE:
						goToPoint( currentPath.getEndPoint( ).getX( ), currentPath.getEndPoint( ).getY( ), targetHeading );
						break;
				}

				break;
			case MANUAL:
				break;
		}
	}
	public void update() {
		pollNormalizedPowerMultiplier();
//		normalizedPowerMultiplier = 12.0 / photonVoltageSensor.getCachedVoltage();
		clearBulkCache( );
		updateTracker( );
		calculateHz();
		rgbController.update();
		intake.update();
		updateDrive();
//		telemetry.update();
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

	public void pollNormalizedPowerMultiplier() {
		if (voltagePollTimer.seconds() > 4) {
			voltagePollTimer.reset( );
			normalizedPowerMultiplier = 12.0 / hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS );
		}
	}

	public static void calculateHz() {
		loop = System.nanoTime( );
		currentHz = 1000000000 / (loop - prevTime);
		prevTime = loop;
	}

	public static double getCurrentHz( ) {
		return currentHz;
	}

	public Vector2D getVelocityVector() {
		return tracker.getDeltaPositionVector().scalarMultiply( currentHz );
	}

	public double getVelocity() {
		return getVelocityVector().getMagnitude();
	}
	public Vector2D getCentripetalForceVector( double curvature ) {
		Vector2D velocityVector = getVelocityVector();

		return velocityVector.vectorMultiply( velocityVector ).scalarMultiply( robotMass ).scalarMultiply( curvature );
	}

	public void setDriveControlState( DriveControlState driveControlState ) {
		this.driveControlState = driveControlState;
	}

	public void setMaxAutoDriveSpeed( double maxAutoDriveSpeed ) {
		this.maxAutoDriveSpeed = maxAutoDriveSpeed;
	}

	public DriveControlState getDriveControlState( ) {
		return driveControlState;
	}
}
