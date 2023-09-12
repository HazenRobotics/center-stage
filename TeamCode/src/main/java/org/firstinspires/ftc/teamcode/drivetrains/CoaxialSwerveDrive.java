package org.firstinspires.ftc.teamcode.drivetrains;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;

public class CoaxialSwerveDrive implements Drive {

	AxonSwervePod[] swervePods = new AxonSwervePod[4];
	double wheelbase;
	double trackwidth;


	public CoaxialSwerveDrive( HardwareMap hw ) {
		this( hw, new String[]{ "FLM", "BLM", "FRM", "BRM" }, new boolean[]{ false, false, false, false },
				new String[]{ "FLS", "BLS", "FRS", "BRS" }, new boolean[]{ false, false, false, false },
				new String[]{ "FLE", "BLE", "FRE", "BRE" }, new double[]{6.15, 4.14, 5.48, 4.85},
				3.3, new boolean[]{ false, false, false, false },
				11.3125, 11.3125, new double[]{ 0.5, 0, 0.015 }, 28 * 8 );
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
	 * @param inverted            true if pods are inverted in order of FL, BL, FR, BR
	 * @param wheelbase           length of base (distance from front wheels to back wheels)
	 * @param trackwidth          width of track (distance from left wheels to right wheels)
	 */
	public CoaxialSwerveDrive( HardwareMap hw, String[] motorNames, boolean[] motorReverse, String[] servoNames, boolean[] servoReversed, String[] servoEncoderNames, double[] servoEncoderOffsets, double servoEncoderVoltage, boolean[] inverted, double wheelbase, double trackwidth, double[] PID, double wheelPPR ) {
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
//			double distanceToTurn = wheelAngles[i] - swervePods[i].getAngle( );
//			if( distanceToTurn > PI ) {
//				wheelAngles[i] -= PI;
//				wheelSpeeds[i] *= -1;
//			}

			swervePods[i].setAngleTarget( wheelAngles[i] );
			swervePods[i].update( wheelSpeeds[i] );
		}
	}

	public void naiveDrive( double power, double angle ) {
		for( int i = 0; i < 4; i++ ) {
			swervePods[i].setAngleTarget( angle );
			swervePods[i].update( power );
		}
	}

	public void spinny( double power ) {
		swervePods[0].setAngleTarget( PI / 4 );
		swervePods[1].setAngleTarget( 3 * PI / 4 );
		swervePods[2].setAngleTarget( 7 * PI / 4 );
		swervePods[3].setAngleTarget( 5 * PI / 4 );

		for( int i = 0; i < 4; i++ )
			swervePods[i].update( power );
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
