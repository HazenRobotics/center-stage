package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BreakBeam {
    DigitalChannel breakBeam;
    public BreakBeam( HardwareMap hw, String breakBeamName) {
        breakBeam = hw.get(DigitalChannel.class, breakBeamName);
        breakBeam.setMode( DigitalChannel.Mode.INPUT );
    }
    public boolean getState() { return breakBeam.getState();}

    public String toString() {
        return Boolean.toString( breakBeam.getState() );
    }
}
