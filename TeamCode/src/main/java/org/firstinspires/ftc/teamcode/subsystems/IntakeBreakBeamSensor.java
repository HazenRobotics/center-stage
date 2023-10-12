package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeBreakBeamSensor {
    DigitalChannel breakBeam;
    Telemetry telemetry;
    boolean connected;

    public IntakeBreakBeamSensor(HardwareMap hw, Telemetry t, String breakBeamName) {
        breakBeam = hw.get(DigitalChannel.class, breakBeamName);
        telemetry = t;
    }

    public void readBeamState() {
        connected = breakBeam.getState();
    }

    public boolean getBeamState() {
        return connected;
    }

}
