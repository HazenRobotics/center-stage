package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.utils.RGBLights;

public class RGBcontroller extends RGBLights{


    private static final com.qualcomm.robotcore.hardware.HardwareMap HardwareMap = ;
    RGBLights rgbSignal;

    public RGBcontroller(String name)
    {
        super(HardwareMap, name);

    }

    public void setColor(RevBlinkinLedDriver.BlinkinPattern Color) {rgbSignal.setPattern(Color);}


    public void autoColor()
    {
       // change colors every 5 seconds
        // private static int everyLoop;

            rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);




    }



}
