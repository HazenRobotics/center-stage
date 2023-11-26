package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.roadrunner.KhepriSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.BreakBeamSensor;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SlingshotLauncher;

import java.util.List;

public class KhepriBot {
	HardwareMap hw;
	Telemetry telemetry;

	public CoaxialSwerveDrive drive;
	public KhepriSwerveDrive roadrunnerDrive;
	public Lift lift;
	public Deposit deposit;
	public Intake intake;
	public Climber climber;
	public SlingshotLauncher launcher;
	public IMU imu;
	public List<LynxModule> hubs;
	private BreakBeamSensor bottom,top,in,ramp;

	public KhepriBot ( HardwareMap hw, Telemetry t) {
		bottom = new BreakBeamSensor(hw,t,"BB-Bot");
		top = new BreakBeamSensor(hw,t,"BB-Top");
		in = new BreakBeamSensor(hw,t,"BB-In");
		ramp = new BreakBeamSensor(hw,t,"BB-Ramp");
		this.hw = hw;
		telemetry = new MultipleTelemetry( t, FtcDashboard.getInstance( ).getTelemetry( ) );

		hubs = hw.getAll( LynxModule.class );
		for( LynxModule hub : hubs ) hub.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

		roadrunnerDrive = new KhepriSwerveDrive(hw);

		drive = new CoaxialSwerveDrive( hw );
		lift = new Lift( hw );
		deposit = new Deposit( hw );
		intake = new Intake( hw, t );
		intake.foldIntake();
		climber = new Climber( hw );
		launcher = new SlingshotLauncher( hw );

		setupIMU( RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD );
	}
	public void setupIMU( RevHubOrientationOnRobot.LogoFacingDirection logoDir, RevHubOrientationOnRobot.UsbFacingDirection usbDir ) {
		imu = hw.get( IMU.class, "imu" );
		imu.initialize( new IMU.Parameters( new RevHubOrientationOnRobot(
				logoDir, usbDir) ) );
	}

	public double getRobotCurrentAmps() {
		return hubs.get( 0 ).getCurrent( CurrentUnit.AMPS ) + hubs.get( 1 ).getCurrent( CurrentUnit.AMPS );
	}
	public boolean isIntakeClear() {
		return getBBin() && getBBRamp();
	}
	public boolean bucketFull() {
		return getBBTop() && getBBBottom();
	}
	public boolean getBBin() {
		return in.getBeamState();
	}
	public boolean getBBTop() {
		return top.getBeamState();
	}
	public boolean getBBBottom() {
		return bottom.getBeamState();
	}
	public boolean getBBRamp() {
		return ramp.getBeamState();
	}

}
