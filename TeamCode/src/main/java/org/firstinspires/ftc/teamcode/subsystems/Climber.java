package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {

	DcMotorEx motor;

	public Climber (HardwareMap hw) {
		this(hw, "climb", true);
	}

	public Climber ( HardwareMap hw, String motorName, boolean motorReversed ) {
		motor = hw.get( DcMotorEx.class, motorName );
		if (motorReversed) motor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void setPower(double power) {
		motor.setPower( power );
	}
}
