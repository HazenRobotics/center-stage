package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

public class Climber {

	DcMotorEx motor;
	public Climber (HardwareMap hw) {
		this(hw, "climb", true);
	}

	public Climber ( HardwareMap hw, String motorName, boolean motorReversed ) {
		motor = new CachingDcMotorEX( hw.get( DcMotorEx.class, motorName ) );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		if (motorReversed) motor.setDirection( DcMotorSimple.Direction.REVERSE );

	}

	public void setPower(double power) {
		motor.setPower( power );
	}
	public void goUp( ) {
		motor.setPower( motor.getPower() == 1 ? 0 : 1 );
	}
	public void goDown( ) {
		motor.setPower( motor.getPower() == -1 ? 0 : -1 );
	}

	public double getPosition() {
		return motor.getCurrentPosition();
	}

	public double getPower() {
		return motor.getPower();
	}

	public DcMotorSimple.Direction getDirection() {
		return motor.getDirection();
	}
}
