package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalLidarSensor {
    DigitalChannel distanceSensor;
    public DigitalLidarSensor( HardwareMap hw, String breakBeamName) {
        distanceSensor = hw.get(DigitalChannel.class, breakBeamName);
        distanceSensor.setMode( DigitalChannel.Mode.INPUT );
    }
    public boolean isBlocked() { return !distanceSensor.getState(); }

}
