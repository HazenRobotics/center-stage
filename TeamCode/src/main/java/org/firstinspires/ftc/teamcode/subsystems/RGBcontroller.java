package org.firstinspires.ftc.teamcode.subsystems;

import static android.icu.lang.UProperty.MATH;
import static android.icu.lang.UProperty.PATTERN_SYNTAX;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.RGBLights;


public class RGBcontroller extends RGBLights {


    ElapsedTime secondsPassed = new ElapsedTime();
    BlinkinPattern[] rgbArray;

    public RGBcontroller(String name, HardwareMap hw) {
        super(hw, name);
        rgbArray = new BlinkinPattern[]{BlinkinPattern.BLUE, BlinkinPattern.BLUE};


    }

    //Take in two parameters of color and alternate between them every second
    public void setColor(BlinkinPattern color) {
        super.setPattern(color);

    }

    public void setColor(BlinkinPattern color0, BlinkinPattern color1) {
        secondsPassed.reset();
        rgbArray[0] = color0;
        rgbArray[1] = color1;
    }

    public void update() {
       super.setPattern(rgbArray[secondsPassed.seconds() % 1 < 0.5 ? 0 : 1]);
    }
}
