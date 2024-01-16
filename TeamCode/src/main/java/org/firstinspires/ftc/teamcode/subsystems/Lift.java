package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

public class Lift {
	public DcMotorEx motor;
	Encoder encoder;
	public PIDController controller;
	public int target = 0;

	public Lift( HardwareMap hardwareMap ) {
		this( hardwareMap, "lift", false, "FRM/liftEnc", new PIDController( 0.02, 0, 0.001 ) );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param motorName   the name of the lift motor in the hardware map
	 */
	public Lift( HardwareMap hardwareMap, String motorName, boolean reverseMotor, String encoderName, PIDController controller ) {
		motor = new CachingDcMotorEX( hardwareMap.get( DcMotorEx.class, motorName ) );
		motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
		encoder = new Encoder( hardwareMap.get( DcMotorEx.class, encoderName ) );
		encoder.reset( );

		if( reverseMotor ) {
			motor.setDirection( DcMotorSimple.Direction.REVERSE );
			encoder.setDirection( Encoder.Direction.REVERSE );
		}

		this.controller = controller;

		resetLift( );
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
	public void updatePID( ) {
		motor.setPower( controller.calculate( getPosition(), target ) * KhepriBot.normalizedPowerMultiplier );
	}
	public void setPower( double power ) {
		motor.setPower( power );
	}

	public double getPower( ) {
		return motor.getPower( );
	}
	public int getTarget( ) {
		return target;
	}

	public int getPosition( ) {
		return -encoder.getCurrentPosition( );
	}

	public double getCurrent( CurrentUnit currentUnit ) {
		return motor.getCurrent( currentUnit );
	}
}
