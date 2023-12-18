package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.SwervePDController;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingCRServo;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingDcMotorEX;

public class AxonSwervePod {

	public DcMotorEx motor;
	CRServo crServo;
	AxonAbsolutePositionEncoder encoder;
	SwervePDController controller;

	double motorPPR;

	public AxonSwervePod( HardwareMap hw, String motorName, String servoName, String encoderName ) {
		this( hw, motorName, false, servoName, false, encoderName,
				0, 3.3, new double[]{ 0, 0 }, 28 * 8 );
	}

	public AxonSwervePod( HardwareMap hw, String motorName, boolean motorReversed, String servoName, boolean servoReversed,
						  String encoderName, double encoderOffset, double encoderVoltage, double[] pid, double PPR ) {
		motor = new CachingDcMotorEX( hw.get( DcMotorEx.class, motorName ) );
		if( motorReversed ) reverseMotor( );

		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

		crServo = new CachingCRServo( hw.get( CRServo.class, servoName ) );
		if( servoReversed ) crServo.setDirection( DcMotorSimple.Direction.REVERSE );

		encoder = new AxonAbsolutePositionEncoder( hw, encoderName, encoderOffset, encoderVoltage );
		controller = new SwervePDController( pid[0], pid[1] );

		motorPPR = PPR;
	}

	public void reverseMotor( ) {
		if( motor.getDirection( ) == DcMotorSimple.Direction.FORWARD )
			motor.setDirection( DcMotorSimple.Direction.REVERSE );
		else
			motor.setDirection( DcMotorSimple.Direction.FORWARD );
	}

	public void reverseServo( ) {
		if( crServo.getDirection( ) == DcMotorSimple.Direction.FORWARD )
			crServo.setDirection( DcMotorSimple.Direction.REVERSE );
		else
			crServo.setDirection( DcMotorSimple.Direction.FORWARD );
	}

	public void setDrivePower( double power ) {
		motor.setPower( power );
	}

	public void setRotatePower( double power ) {
		crServo.setPower( power );
	}

	public void setOffset( double offset ) {
		encoder.setOffset( offset );
	}

	public double getOffset( ) {
		return encoder.getOffset( );
	}

	public void setAngleTarget( double target ) {
		controller.setTargetAngle( target );
	}

	public double getAngle( ) {
		return encoder.getAngle( );
	}

	public double getPIDError( ) {
		return controller.getError( );
	}

	public double getDriveVelo( ) {
		return motor.getVelocity( );
	}

	public int getDrivePosition( ) {
		return motor.getCurrentPosition( );
	}

	public double getDriveAmp( ) {
		return motor.getCurrent( CurrentUnit.AMPS );
	}

	public double getServoAmp( ) {
		return motor.getCurrent( CurrentUnit.AMPS );
	}

	public void setPID( double p, double d ) {
		controller.setPD( p, d );
	}

	public void setKs( double s ) {
		controller.setKs( s );
	}

	public double getError( ) {
		return controller.getError( );
	}

	public void update( ) {
		double[] results = controller.update( getAngle( ) );

		setRotatePower( results[0] );
	}

	public void update( double motorPower ) {
		double[] results = controller.update( getAngle( ) );

		setRotatePower( results[0] );
		setDrivePower( results[1] * motorPower );
	}


}
