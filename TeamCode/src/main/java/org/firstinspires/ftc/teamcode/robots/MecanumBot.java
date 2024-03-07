package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelLauncher;

public class MecanumBot {

	public MecanumDrive drive;
	public MecanumBot( HardwareMap hw ) {
		drive = new MecanumDrive( hw );
	}
	public MecanumBot( HardwareMap hw, String[] drivetrainNames ) {
		drive = new MecanumDrive(hw, drivetrainNames, new boolean[]{false, false, true, true});
	}

}
