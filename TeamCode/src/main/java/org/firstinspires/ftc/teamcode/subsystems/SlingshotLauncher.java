package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingServo;

public class SlingshotLauncher {

	Servo servo;

	public SlingshotLauncher ( HardwareMap hw ) {
		servo = new CachingServo( hw.get(Servo.class, "droneLaunch" ) );
		prime();
	}

	public void prime() {
		servo.setPosition( 0.52 );
	}

	public void release() {
		servo.setPosition( 0.39 );
	}

	public double getPosition() {
		return servo.getPosition();
	}

}
