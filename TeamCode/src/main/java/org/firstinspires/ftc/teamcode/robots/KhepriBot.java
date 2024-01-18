package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.utils.SwervePDController.findShortestAngularTravel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
	public Vector2 currentPointTarget;

	static double loop, loopTime, currentHz, prevTime;

	public static final double robotMass = 34.1;

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
		autoHeadingController = new PIDController( 5, 0, 0.4 );
		teleOpHeadingController = new PIDController( 1, 0, 0.1 );

		voltagePollTimer = new ElapsedTime();
		normalizedPowerMultiplier = 12.0 / hubs.get( 0 ).getInputVoltage( VoltageUnit.VOLTS );
//		photonVoltageSensor = hw.getAll(PhotonLynxVoltageSensor.class).iterator().next();
//		normalizedPowerMultiplier = 12.0 / photonVoltageSensor.getCachedVoltage();
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

	public void setupTeleOpTracker( Pose2D startPose ) {
		tracker = new ThreeWheelTracker(
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
				new Encoder( hw.get( DcMotorEx.class, "BLM/perpEnc" ) ).setDirection( Encoder.Direction.FORWARD )

		);
		updateTracker();
	}

	public Pose2D getPose() {
		return poseEstimate;
	}

	public void updateTracker () {
		tracker.updatePose();
		poseEstimate = tracker.getPose2D().add( 0, 0, new AngleDegrees( 90 ) );
	}

	public void goToPoint( double x, double y, double heading, double travelMultiplier, double rotationMultiplier ) {
		driveControlState = DriveControlState.P2P;
		currentPointTarget = new Vector2( x, y );
		telemetry.addData( "pose", poseEstimate );
		telemetry.addData( "target", "X: " + x + " Y: " + y + " heading: " + heading );
		double headingError = findShortestAngularTravel( Math.toRadians( heading ), poseEstimate.getTheta().getRadians() );
		telemetry.addData( "heading error", headingError );

		double yPow = YController.calculate(poseEstimate.getY(), y);
		double xPow = XController.calculate(poseEstimate.getX(), x);

		if (Math.hypot( YController.getPositionError(), XController.getPositionError() ) > 1) drive.setMaxSpeed( 0.65 );
		else drive.setMaxSpeed( 1 );


		drive.fieldCentricDrive(
				yPow * normalizedPowerMultiplier,
				xPow * normalizedPowerMultiplier,
				autoHeadingController.calculate( headingError, 0 ) * normalizedPowerMultiplier,
				poseEstimate.getTheta().getRadians()
		);
	}

	public void goToPoint( double x, double y, double heading ) {
		goToPoint( x, y, heading, 1, 1 );
	}

	public void goToPoint( Pose2D pose ) {
		goToPoint( pose.getX(), pose.getY(), pose.getTheta().getDegrees(), 1, 1 );
	}

	public void goToPoint( Pose2D pose, double travelMultiplier, double rotationMultiplier ) {
		goToPoint( pose.getX(), pose.getY(), pose.getTheta().getDegrees(), travelMultiplier, rotationMultiplier );
	}

	public void followPath( GVFPath path, double targetHeading ) {
		driveControlState = DriveControlState.GVF;
		currentPath = path;
		Vector2 currentPos = new Vector2( poseEstimate.getX( ), poseEstimate.getY( ) );
		double headingError = findShortestAngularTravel( Math.toRadians( targetHeading ), poseEstimate.getTheta( ).getRadians( ) );

		if (path.getDistanceFromEnd( currentPos ) > 1) drive.setMaxSpeed( 0.65 );
		else drive.setMaxSpeed( 1 );

		switch( path.evaluateState( currentPos ) ) {
			case FOLLOW_PATH:
				Vector2 powerVector = path.calculateGuidanceVector( currentPos );
				drive.fieldCentricDrive(
						powerVector.getY( ) * normalizedPowerMultiplier,
						powerVector.getX( ) * normalizedPowerMultiplier,
						teleOpHeadingController.calculate( headingError, 0 ),
						poseEstimate.getTheta( ).getRadians( )
				);
				break;
			case USE_PID:
			case DONE:
				goToPoint( path.getEndPoint( ).getX( ), path.getEndPoint( ).getY( ), targetHeading );
				break;
		}
	}

	public GVFPath getCurrentPath( ) {
		return currentPath;
	}

	public Vector2 getCurrentPointTarget( ) {
		return currentPointTarget;
	}

	public void update() {
		pollNormalizedPowerMultiplier();
//		normalizedPowerMultiplier = 12.0 / photonVoltageSensor.getCachedVoltage();
		clearBulkCache( );
		updateTracker( );
		calculateHz();
		rgbController.update();
		intake.update();
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

	public Vector2D getCentripetalForceVector( double curvature ) {
		Vector2D velocityVector = tracker.getDeltaPositionVector().scalarMultiply( currentHz );

		return velocityVector.vectorMultiply( velocityVector ).scalarMultiply( robotMass ).scalarMultiply( curvature );
	}

	public DriveControlState getDriveControlState( ) {
		return driveControlState;
	}
}
