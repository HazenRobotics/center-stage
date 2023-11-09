package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RGBcontroller {
    RevBlinkinLedDriver rgbSignal;
    RevBlinkinLedDriver.BlinkinPattern[] rgbArray = {RevBlinkinLedDriver.BlinkinPattern.WHITE,RevBlinkinLedDriver.BlinkinPattern.YELLOW ,
            RevBlinkinLedDriver.BlinkinPattern.GREEN,RevBlinkinLedDriver.BlinkinPattern.HOT_PINK };

    public RGBcontroller(HardwareMap hw, String name)
    {
        rgbSignal = hw.get(RevBlinkinLedDriver.class, name);

    }

    public RevBlinkinLedDriver.BlinkinPattern selectColor(int num)
     {
         return rgbArray[num];

     }

     public void turnOffColor()
     {

     }


}
