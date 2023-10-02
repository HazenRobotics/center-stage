package org.firstinspires.ftc.teamcode.drivetrains;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

public class CoaxialSwerveDrive {

	public enum WheelState {
		DRIVE,
		DIAMOND,
		X
	}

	WheelState wheelState = WheelState.DRIVE;
	AxonSwervePod[] swervePods = new AxonSwervePod[4];
	public static final double[] encoderOffsets = { 3.47, 4.24, 0.94, 0.04 };
	double wheelbase;
	double trackwidth;
	double[] wheelSpeeds;
	double[] wheelAngles;

	public CoaxialSwerveDrive( HardwareMap hw ) {
		this( hw, new String[]{ "FLM/perp", "BLM", "FRM", "BRM/para" }, new boolean[]{ false, false, false, false },
				new String[]{ "FLS", "BLS", "FRS", "BRS" }, new boolean[]{ false, false, false, false },
				new String[]{ "FLE", "BLE", "FRE", "BRE" }, encoderOffsets,
				3.3, 11.3125, 11.3125, new double[]{ 0.6, 0.02 }, 28 * 8 );
	}

	/**
	 * @param hw                  robot's hardware map
	 * @param motorNames          motor names in order of FL, BL, FR, BR
	 * @param motorReverse        true if motor reversed in order of FL, BL, FR, BR
	 * @param servoNames          servo names in order of FL, BL, FR, BR
	 * @param servoReversed       servo encoder names in order of FL, BL, FR, BR
	 * @param servoEncoderNames   true if servo reversed in order of FL, BL, FR, BR
	 * @param servoEncoderOffsets servo encoder offsets in order of FL, BL, FR, BR
	 * @param servoEncoderVoltage maximum voltage of encoder used for servos
	 * @param wheelbase           length of base (distance from front wheels to back wheels)
	 * @param trackwidth          width of track (distance from left wheels to right wheels)
	 */
	public CoaxialSwerveDrive( HardwareMap hw, String[] motorNames, boolean[] motorReverse, String[] servoNames, boolean[] servoReversed, String[] servoEncoderNames, double[] servoEncoderOffsets, double servoEncoderVoltage, double wheelbase, double trackwidth, double[] PID, double wheelPPR ) {
		for( int i = 0; i < swervePods.length; i++ )
			swervePods[i] = new AxonSwervePod( hw, motorNames[i], motorReverse[i], servoNames[i], servoReversed[i],
					servoEncoderNames[i], servoEncoderOffsets[i], servoEncoderVoltage, PID, wheelPPR );

		this.wheelbase = wheelbase;
		this.trackwidth = trackwidth;
	}

	/**
	 * field oriented swerve drive
	 *
	 * @param drivePower  power to move in the y direction (relative to the field)
	 * @param strafePower power to move in the x direction (relative to the field)
	 * @param rotatePower power to turn the robot (right)
	 */
	public void drive( double drivePower, double strafePower, double rotatePower ) {
		boolean rotatePods = Math.abs( drivePower ) > 0.02 ||
				Math.abs( strafePower ) > 0.02 ||
				Math.abs( rotatePower ) > 0.02;

		switch( wheelState ) {
			case DRIVE:
				// distance between opposite wheels
				double R = Math.sqrt( wheelbase * wheelbase + trackwidth * trackwidth );

				double A = strafePower - rotatePower * (wheelbase / R);
				double B = strafePower + rotatePower * (wheelbase / R);
				double C = drivePower + rotatePower * (trackwidth / R);
				double D = drivePower - rotatePower * (trackwidth / R);

				double ws1 = Math.sqrt( B * B + D * D );
				double wa1 = Math.atan2( B, D ) - PI / 2;
				double ws2 = Math.sqrt( A * A + D * D );
				double wa2 = Math.atan2( A, D ) - PI / 2;
				double ws3 = Math.sqrt( B * B + C * C );
				double wa3 = Math.atan2( B, C ) - PI / 2;
				double ws4 = Math.sqrt( A * A + C * C );
				double wa4 = Math.atan2( A, C ) - PI / 2;

				double max = Math.max( Math.max( ws1, ws2 ), Math.max( ws3, ws4 ) );

				if( max > 1 ) {
					ws1 /= max;
					ws2 /= max;
					ws3 /= max;
					ws4 /= max;
				}

				wheelSpeeds = new double[]{ ws1, ws2, ws3, ws4 };
				wheelAngles = new double[]{ wa1, wa2, wa3, wa4 };
				break;
			case DIAMOND:
				wheelSpeeds = new double[]{ 0, 0, 0, 0 };
				wheelAngles = new double[]{ PI / 4, 3 * PI / 4, 3 * PI / 4, PI / 4 };
				rotatePods = true;
				break;
			case X:
				wheelSpeeds = new double[]{ 0, 0, 0, 0 };
				wheelAngles = new double[]{ 3 * PI / 4, PI / 4, PI / 4, 3 * PI / 4 };
				rotatePods = true;
				break;
		}

		for( int i = 0; i < swervePods.length; i++ ) {
			if( rotatePods )
				swervePods[i].setAngleTarget( wheelAngles[i] );
			swervePods[i].update( wheelSpeeds[i] );
		}
	}

	public void fieldCentricDrive( double drivePower, double strafePower, double rotatePower, double heading ) {
		double temp = drivePower * Math.cos( heading ) + strafePower * Math.sin( heading );
		strafePower = -drivePower * Math.sin( heading ) + strafePower * Math.cos( heading );
		drivePower = -temp;

		drive( drivePower, strafePower, rotatePower );
	}

	public void setWheelState( WheelState state ) {
		wheelState = state;
	}

	public WheelState getWheelState( ) {
		return wheelState;
	}
}
