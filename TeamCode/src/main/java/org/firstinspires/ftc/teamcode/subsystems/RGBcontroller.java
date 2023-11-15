package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.RGBLights;

public class RGBcontroller extends RGBLights{

    private static final com.qualcomm.robotcore.hardware.HardwareMap HardwareMap = ;
    RGBLights rgbSignal;


    public RGBcontroller(String name)
    {
        super(HardwareMap, name);

    }//

    public void setColor(RevBlinkinLedDriver.BlinkinPattern Color) {
        rgbSignal.setPattern(Color);
    }


}
