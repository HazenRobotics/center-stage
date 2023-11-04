package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlingshotLauncher {

	Servo servo;

	public SlingshotLauncher ( HardwareMap hw ) {
		servo = hw.get(Servo.class, "droneLaunch" );
		servo.setPosition( 0.3 );
	}

	public void release() {
		servo.setPosition( 0 );
	}

	public double getPosition() {
		return servo.getPosition();
	}

}
