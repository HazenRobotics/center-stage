package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.RGBLights;

public class RGBcontroller extends RGBLights{


    RGBLights rgbSignal;

    public RGBcontroller(String name,HardwareMap hw)
    {
        super(hw, name);

    }

    public void setColor(RevBlinkinLedDriver.BlinkinPattern Color) {rgbSignal.setPattern(Color);}


    public void autoColor()
    {
       /* NOT COMPLETE
         change colors every 5 second
        private static int everyLoop; */

            rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);




    }



}
