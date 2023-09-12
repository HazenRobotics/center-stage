package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.SwervePIDController;

public class AxonSwervePod {

	DcMotorEx motor;
	CRServo crServo;
	AxonAbsolutePositionEncoder encoder;
	SwervePIDController controller;

	double motorPPR;

	public AxonSwervePod(HardwareMap hw, String motorName, String servoName, String encoderName ) {
		this(hw, motorName, false, servoName, false, encoderName,
				0, 3.3, false, new double[]{0,0,0}, 28 * 8 );
	}

	public AxonSwervePod(HardwareMap hw, String motorName, boolean motorReversed, String servoName, boolean servoReversed,
                         String encoderName, double encoderOffset, double encoderVoltage, boolean inverted, double[] pid, double PPR ) {
		motor = hw.get( DcMotorEx.class, motorName );
		if( motorReversed ) reverseMotor( );
		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		crServo = hw.get( CRServo.class, servoName );
		if( servoReversed ) crServo.setDirection( DcMotorSimple.Direction.REVERSE );

		encoder = new AxonAbsolutePositionEncoder( hw, encoderName, encoderOffset, encoderVoltage, inverted );
		controller = new SwervePIDController( pid[0], pid[1], pid[2] );

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

	public double getOffset() {
		return encoder.getOffset();
	}

	public void setAngleTarget( double target ) {
		controller.setTargetAngle( target );
	}
	public double getAngle( ) {
		return encoder.getAngle( );
	}
	public double getPIDError( ) {
		return controller.getError();
	}
	public double getDriveVelo( ) {
		return motor.getVelocity( );
	}

	public double getDriveAmp( ) {
		return motor.getCurrent( CurrentUnit.AMPS );
	}

	public double getServoAmp( ) {
		return motor.getCurrent( CurrentUnit.AMPS );
	}

	public void setPID( double p, double i, double d) {
		controller.setPID( p, i, d);
	}
	public void update( double motorPower ) {
		setDrivePower( motorPower );
		setRotatePower( controller.update( getAngle() ) );
	}


}
