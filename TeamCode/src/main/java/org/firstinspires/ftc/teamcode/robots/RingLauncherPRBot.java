package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelLauncher;

public class RingLauncherPRBot {

	public MecanumDrive drive;
	public FlywheelLauncher launcher;

	public RingLauncherPRBot( HardwareMap hw ) {
		drive = new MecanumDrive( hw );
		launcher = new FlywheelLauncher( hw );
	}
	public RingLauncherPRBot( HardwareMap hw, String[] drivetrainNames, String[] flywheelNames, String servoName ) {
		drive = new MecanumDrive(hw, drivetrainNames, new boolean[]{false, false, true, true});
		launcher = new FlywheelLauncher(hw, flywheelNames, new boolean[]{true, false}, servoName);
	}

}
