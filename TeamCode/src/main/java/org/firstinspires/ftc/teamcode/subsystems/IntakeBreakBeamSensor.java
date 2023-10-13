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
        breakBeam.setMode( DigitalChannel.Mode.INPUT );
        telemetry = t;
    }

    public void updateBeamState() {
        connected = breakBeam.getState();
    }

    public boolean getBeamState() {
        return connected;
    }

    public void addTelemetry() {
        telemetry.addData( "mode", breakBeam.getMode() );
        telemetry.addData( "state", breakBeam.getState() );
    }
}
