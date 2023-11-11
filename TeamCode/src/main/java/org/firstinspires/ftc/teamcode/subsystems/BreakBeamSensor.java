package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BreakBeamSensor {
    String name;
    DigitalChannel breakBeam;
    Telemetry telemetry;

    public BreakBeamSensor(HardwareMap hw, Telemetry t, String breakBeamName) {
        breakBeam = hw.get(DigitalChannel.class, breakBeamName);
        breakBeam.setMode( DigitalChannel.Mode.INPUT );
        name = breakBeamName;
        telemetry = t;
    }

    public boolean getBeamState() {
        return breakBeam.getState();
    }

    public void addTelemetry() {
        telemetry.addData( name+" state", breakBeam.getState() );
    }
}
