package org.firstinspires.ftc.teamcode.subsystems;

import static android.icu.lang.UProperty.MATH;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RGBLights;


public class RGBcontroller extends RGBLights{


    RGBLights rgbSignal;

    public RGBcontroller(String name,HardwareMap hw)
    {
        super(hw, name);

    }

    public void setColor(RevBlinkinLedDriver.BlinkinPattern Color) {rgbSignal.setPattern(Color);}


    public void autoColor() throws InterruptedException {
       /* NOT COMPLETE
         change colors every 5 second
        */
        boolean isBlue = true;
        int alternateChange = 0;
        ElapsedTime secondsPassed = new ElapsedTime();
        while(alternateChange < 6) {

            if (secondsPassed.milliseconds() >= 5 * Math.pow(10, 3)) {
                if (isBlue)
                    rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                isBlue = false;
            }else {
                rgbSignal.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                isBlue = true;
            }
              alternateChange++;
              secondsPassed.reset();
        }


    }



}
