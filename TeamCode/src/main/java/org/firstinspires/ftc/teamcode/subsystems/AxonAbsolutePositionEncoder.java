package org.firstinspires.ftc.teamcode.subsystems;


import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonAbsolutePositionEncoder {

	public static final double TWO_PI = 2 * PI;

	AnalogInput encoder;
	double angularOffset, maxVoltage;
	int inverted;

	public AxonAbsolutePositionEncoder(HardwareMap hw ) {
		this(hw, "absoluteEncoder");
	}

	public AxonAbsolutePositionEncoder(HardwareMap hw, String encoderName ) {
		this(hw, encoderName, 0, 3.3);
	}

	public AxonAbsolutePositionEncoder(HardwareMap hw, String encoderName, double offset, double volt ) {
		encoder = hw.analogInput.get( encoderName );
		angularOffset = offset;
		maxVoltage = volt;

	}

	public void setOffset(double offset) {
		angularOffset = offset;
	}

	public double getOffset() {
		return angularOffset;
	}

	public double getVoltage() {
		return encoder.getVoltage();
	}

	public double getAngle() {
		return TWO_PI - ((((getVoltage() / maxVoltage) * TWO_PI ) + angularOffset + TWO_PI) % TWO_PI);
	}

}
