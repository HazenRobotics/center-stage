package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingServo;

public class SlingshotLauncher {

	Servo servo;

	public SlingshotLauncher ( HardwareMap hw ) {
		servo = new CachingServo( hw.get(Servo.class, "droneLaunch" ) );
		servo.setPosition( 0 );
	}

	public void release() {
		servo.setPosition( 1 );
	}

	public double getPosition() {
		return servo.getPosition();
	}

}
