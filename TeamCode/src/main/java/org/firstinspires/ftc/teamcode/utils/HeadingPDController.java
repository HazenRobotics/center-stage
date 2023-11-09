package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.subsystems.AxonAbsolutePositionEncoder.TWO_PI;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.util.ElapsedTime;

public class HeadingPDController {


	double Kp, Kd, lastError, error;
	ElapsedTime timer;

	double targetHeading;

	public HeadingPDController( ) {
		this( 0, 0 );
	}

	public HeadingPDController( double p, double d ) {
		setPD( p, d );
		timer = new ElapsedTime( );
		targetHeading = 0;
	}

	public void setPD( double p, double d ) {
		Kp = p;
		Kd = d;
	}

	public double update( double currentAngle ) {
		error = findShortestAngularTravel( targetHeading, currentAngle );

		double derivative = (error - lastError) / timer.seconds( );
		lastError = error;

		timer.reset( );

		return (Kp * error) + (Kd * derivative);
	}

	public void setTargetHeading( double heading ) {
		targetHeading = heading;
	}

	public double getError( ) {
		return error;
	}

	public static double findShortestAngularTravel( double targetAngle, double currentAngle ) {
		return ((((targetAngle - currentAngle + PI) % TWO_PI) + TWO_PI) % TWO_PI) - PI;
	}

}
