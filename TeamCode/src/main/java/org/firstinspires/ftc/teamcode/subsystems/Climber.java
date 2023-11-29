package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

public class Climber {

	DcMotorEx motor;
	Encoder encoder;

	public Climber (HardwareMap hw) {
		this(hw, "climb", false, "FRM/climbEnc");
	}

	public Climber ( HardwareMap hw, String motorName, boolean motorReversed, String encoderName ) {
		motor = hw.get( DcMotorEx.class, motorName );
		encoder = new Encoder(hw.get( DcMotorEx.class, encoderName ));
		encoder.reset();

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
		return encoder.getCurrentPosition();
	}

	public double getPower() {
		return motor.getPower();
	}
}
