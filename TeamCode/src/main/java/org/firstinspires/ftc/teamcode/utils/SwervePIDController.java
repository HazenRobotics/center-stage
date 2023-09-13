package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.subsystems.AxonAbsolutePositionEncoder.TWO_PI;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwervePIDController {


	double Kp, Ki, Kd, integralSum, lastError, error;
	ElapsedTime timer;

	double targetAngle;
	double motorDirection;

	public SwervePIDController( ) {
		this( 0, 0, 0 );
	}

	public SwervePIDController( double p, double i, double d ) {
		setPID( p, i, d );
		timer = new ElapsedTime( );
		targetAngle = 0;
	}

	public void setPID( double p, double i, double d ) {
		Kp = p;
		Ki = i;
		Kd = d;
	}

	public double[] update( double currentAngle ) {
		error = findShortestAngularTravel( targetAngle, currentAngle );

		motorDirection = Math.abs( error ) >= (PI / 2) ? -1 : 1;

		if( motorDirection < 0 ) error = findShortestAngularTravel( targetAngle + PI, currentAngle );

		integralSum += error * timer.seconds( );

		double derivative = (error - lastError) / timer.seconds( );
		lastError = error;

		timer.reset( );

		return new double[]{ (Kp * error) + (Ki * integralSum) + (Kd * derivative), motorDirection };
	}

	public void setTargetAngle( double angle ) {
		targetAngle = angle;
	}

	public double getError( ) {
		return error;
	}

	public static double findShortestAngularTravel( double targetAngle, double currentAngle ) {
		return ((((targetAngle - currentAngle + PI) % TWO_PI) + TWO_PI) % TWO_PI) - PI;
	}

}
