package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

	DcMotorEx[] motors = new DcMotorEx[4];
	IMU imu;

	public MecanumDrive( HardwareMap hw ) {
		this(hw, new String[]{"FLM", "BLM", "FRM", "BRM" }, new boolean[]{false, false, true, true});
	}
	public MecanumDrive( HardwareMap hw, String[] motorNames, boolean[] motorsReversed ) {
		for( int i = 0; i < motors.length; i++ ) {
			motors[i] = hw.get( DcMotorEx.class, motorNames[i] );
			motors[i].setDirection( motorsReversed[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
			motors[i].setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
		}
	}

	public void setupIMU( HardwareMap hw, RevHubOrientationOnRobot.LogoFacingDirection logoDir,
						  RevHubOrientationOnRobot.UsbFacingDirection usbDir  ) {
		imu = hw.get(IMU.class, "imu");

		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot( logoDir, usbDir));

		imu.initialize(parameters);
	}

	public void resetIMU() {
		imu.resetYaw();
	}

	public void robotCentricDrive(double drive, double strafe, double rotate) {
		// used for normalizing powers
		double maxPower = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

		double frontLeftPower = (drive - strafe + rotate) / maxPower;
		double backLeftPower = (drive + strafe + rotate) / maxPower;
		double frontRightPower = (drive + strafe - rotate) / maxPower;
		double backRightPower = (drive - strafe - rotate) / maxPower;

		setMotorPowers( new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower} );
	}

	public void fieldCentricDrive(double drive, double strafe, double rotate) {

		double botHeading = imu.getRobotYawPitchRollAngles().getYaw( AngleUnit.RADIANS);

		double adjustedStrafe = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
		double adjustedDrive = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

		robotCentricDrive( adjustedDrive, adjustedStrafe, rotate );
	}

	public void setMotorPowers( double[] powers ){
		for( int i = 0; i < motors.length; i++ ) {
			motors[i].setPower( powers[i] );
		}
	}
}
