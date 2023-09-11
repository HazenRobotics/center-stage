package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

public class CoaxialSwerveDrive implements Drive {

	AxonSwervePod[] swervePods = new AxonSwervePod[4];
	double wheelbase;
	double trackwidth;

	/**
	 * @param hardwareMap         robot's hardware map
	 * @param motorNames          motor names in order of FL, BL, FR, BR
	 * @param motorReverse        true if motor reversed in order of FL, BL, FR, BR
	 * @param servoNames          servo names in order of FL, BL, FR, BR
	 * @param servoReversed       servo encoder names in order of FL, BL, FR, BR
	 * @param servoEncoderNames   true if servo reversed in order of FL, BL, FR, BR
	 * @param servoEncoderOffsets servo encoder offsets in order of FL, BL, FR, BR
	 * @param servoEncoderVoltage maximum voltage of encoder used for servos
	 * @param inverted            true if pods are inverted in order of FL, BL, FR, BR
	 * @param wheelbase           length of base (distance from front wheels to back wheels)
	 * @param trackwidth          width of track (distance from left wheels to right wheels)
	 */
	public CoaxialSwerveDrive(HardwareMap hardwareMap, String[] motorNames, boolean[] motorReverse, String[] servoNames, boolean[] servoReversed, String[] servoEncoderNames, double[] servoEncoderOffsets, double servoEncoderVoltage, boolean[] inverted, double wheelbase, double trackwidth ) {
		for( int i = 0; i < swervePods.length; i++ )
			swervePods[i] = new AxonSwervePod( hardwareMap, motorNames[i], motorReverse[i], servoNames[i], servoReversed[i],
					servoEncoderNames[i], servoEncoderOffsets[i], servoEncoderVoltage, inverted[i], new double[]{ 0, 0, 0 }, 28 * 8 );

		this.wheelbase = wheelbase;
		this.trackwidth = trackwidth;
	}

	/**
	 * field oriented swerve drive
	 *
	 * @param drivePower  power to move in the y direction (relative to the field)
	 * @param strafePower power to move in the x direction (relative to the field)
	 * @param rotatePower power to turn the robot (right)
	 * @param heading     robot's heading (RAD)
	 */
	public void move( double drivePower, double strafePower, double rotatePower, double heading ) {
		double temp = drivePower * Math.cos( heading ) + strafePower * Math.sin( heading );
		strafePower = -drivePower * Math.sin( heading ) + strafePower * Math.cos( heading );
		drivePower = temp;

		// distance between opposite wheels
		double R = Math.sqrt( wheelbase * wheelbase + trackwidth * trackwidth );

		double A = strafePower - rotatePower * (wheelbase / R);
		double B = strafePower + rotatePower * (wheelbase / R);
		double C = drivePower - rotatePower * (trackwidth / R);
		double D = drivePower + rotatePower * (trackwidth / R);

		double ws1 = Math.sqrt( B * B + C * C );
		double wa1 = Math.atan2( B, C );
		double ws2 = Math.sqrt( B * B + D * D );
		double wa2 = Math.atan2( B, D );
		double ws3 = Math.sqrt( A * A + D * D );
		double wa3 = Math.atan2( A, D );
		double ws4 = Math.sqrt( A * A + C * C );
		double wa4 = Math.atan2( A, C );

		double max = Math.max( Math.max( ws1, ws2 ), Math.max( ws3, ws4 ) );

		if( max > 1 ) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}

		double[] wheelSpeeds = new double[]{ ws1, ws2, ws3, ws4 };
		double[] wheelAngles = new double[]{ wa1, wa2, wa3, wa4 };

		for( int i = 0; i < swervePods.length; i++ ) {
			// if angle to turn to is greater than 180, negate power and reduce angle by 180
			double distanceToTurn = wheelAngles[i] - swervePods[i].getAngle( );
			if( distanceToTurn > Math.PI ) {
				wheelAngles[i] -= Math.PI;
				wheelSpeeds[i] *= -1;
			}

			swervePods[i].setDrivePower( wheelSpeeds[i] );
			swervePods[i].setAngleTarget( wheelAngles[i] );
			swervePods[i].update();
		}


	}

	@Override
	public void move( double power ) {

	}

	@Override
	public void turn( double power ) {

	}

	@Override
	public void stop( ) {

	}

	@Override
	public void drive( double move, double turn ) {

	}

	@Override
	public State getState( ) {
		return null;
	}
}
