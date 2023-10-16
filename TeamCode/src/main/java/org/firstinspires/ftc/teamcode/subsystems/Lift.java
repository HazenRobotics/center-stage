package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift {

	double PULSES_PER_REVOLUTION;
	double GEAR_RATIO;

	public DcMotorEx motor;

	static int liftPosition = 0;

	double spoolRadius;
	double posOffset;
	double liftAngle; // the angle of the lift from the ground in 'angleUnit's
	AngleUnit angleUnit; // the angle unit for the lift angle i.e. degrees or radians

	public PIDController controller;
	public int target = 0;

	public Lift( HardwareMap hardwareMap ) {
		this( hardwareMap, "lift", false, 0,
				0.5, 0, AngleUnit.DEGREES, 103.8, 1,
				new PIDController(0,0,0) );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param motorName   the name of the lift motor in the hardware map
	 * @param posOffset   the height of the bottom of the lift to the ground
	 * @param spoolRadius the radius of the spool attached om 'angleUnit's
	 * @param liftAngle   the angle of the lift from the ground in
	 * @param angleUnit   the angle unit to make calculations and input variables
	 */
	public Lift( HardwareMap hardwareMap, String motorName, boolean reverseMotor, double posOffset,
				 double spoolRadius, double liftAngle, AngleUnit angleUnit, double PPR, double gearRatio, PIDController controller ) {
		motor = hardwareMap.get( DcMotorEx.class, motorName );
		motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

		if( reverseMotor )
			reverseMotor();

		setPosOffset( posOffset );
		setSpoolRadius( spoolRadius );
		setLiftAngle( liftAngle );
		setAngleUnit( angleUnit );

		PULSES_PER_REVOLUTION = PPR;
		GEAR_RATIO = gearRatio;

		this.controller = controller;

		resetLift( );
	}

	/**
	 * reverses the motor direction
	 * (if it is FORWARD sets it to REVERSE or if it is REVERSE, sets it to FORWARD)
	 */
	public void reverseMotor( ) {
		motor.setDirection( motor.getDirection( ) == DcMotorSimple.Direction.FORWARD ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
	}

	/**
	 * stops and resets the physical motor and its encoder and sets liftPosition to 0
	 */
	public void resetLift( ) {
		motor.setPower( 0 );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
	}

	public void setTarget( int target ) {
		this.target = target;
	}

	public void setTargetInches( double inches ) {
		setTarget( convertDistTicks( inches, 2 * spoolRadius * Math.PI ) );
	}

	public void updatePID( double multiplier ) {
		motor.setPower( controller.calculate( motor.getCurrentPosition(), target ) * multiplier );
	}

	public void setPIDValues(double p, double i, double d) {
		controller.setPID( p, i, d );
	}


	// converters and calculators

	/**
	 * @param distance      the distance to move in inches
	 * @param circumference the circumference of the wheel that has the encoder
	 * @return the number of ticks in that distance
	 */
	public int convertDistTicks( double distance, double circumference ) {
		return (int) Math.round( ((distance / circumference) * PULSES_PER_REVOLUTION) / GEAR_RATIO );
	}

	/**
	 * @param ticks         the distance to move in ticks
	 * @param circumference the circumference of the wheel that has the encoder
	 * @return the distance in that number of ticks
	 */
	public double convertTicksDist( double ticks, double circumference ) {
		return (ticks * circumference * GEAR_RATIO) / PULSES_PER_REVOLUTION;
	}

	/*
						/|
			 (lift) c  / |
	  		          /  |  b the height of this side of the triangle
(bottom of bucket)   /___|  h (height given from the ground)
      B (this angle) ^   |
					_____|
 		  	       (ground)

		* given h
		* find c (the distance to set the lift to)
		* g is the distance the bucket is off the ground
		* B is the lift angle (55°)

		sin(θ) = b/c
		b = h-g
		θ = B

		c = (h-g)/(sin(B)
		liftPosition = (height - posOffset/( sin(liftAngle) )

	 */


	// getters and setters

	public boolean isBusy( ) {
		return motor.isBusy( );
	}

	// getters and setters for power and velocity

	public double getPower( ) {
		return motor.getPower( );
	}



	// getters for the lift position

	public int getTarget( ) {
		return target;
	}

	public static int getPosition( boolean statics ) {
		return liftPosition;
	}

	public int getMotorPosition( ) {
		return motor.getCurrentPosition( );
	}

	public double getMotorPositionInch( ) {
		return convertTicksDist( getMotorPosition( ), 2 * spoolRadius * Math.PI );
	}

	// setters and getters for angleUnit

	public void setAngleUnit( AngleUnit angleUnit ) {
		this.angleUnit = angleUnit;
	}

	public AngleUnit getAngleUnit( ) {
		return angleUnit;
	}

	// setters and getters for spoolRadius

	public double getSpoolRadius( ) {
		return spoolRadius;
	}

	public void setSpoolRadius( double newRadius ) {
		spoolRadius = newRadius;
	}

	// setters and getters for posOffset
	public void setPosOffset( double height ) {
		posOffset = height;
	}

	public double getPosOffset( ) {
		return posOffset;
	}

	// setters and getters for liftAngle
	public void setLiftAngle( double newAngle ) {
		liftAngle = newAngle;
	}

	public double getLiftAngle( ) {
		return liftAngle;
	}

	/**
	 * @param angle the angle unit to get the lift's angle in
	 * @return the angle of the lift in the angle unit requested
	 */
	public double getLiftAngle( AngleUnit angle ) {

		if( angle.equals( angleUnit ) )
			return liftAngle;
		else if( angle.equals( AngleUnit.RADIANS ) )
			return Math.toRadians( liftAngle );
		else if( angle.equals( AngleUnit.DEGREES ) )
			return Math.toDegrees( liftAngle );
		return liftAngle;
	}

	public double getCurrent( CurrentUnit currentUnit ) {
		return motor.getCurrent( currentUnit );
	}
}
