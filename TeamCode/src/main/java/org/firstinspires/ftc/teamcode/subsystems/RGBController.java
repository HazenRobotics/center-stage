package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RGBLights;


public class RGBController extends RGBLights {


    ElapsedTime secondsPassed = new ElapsedTime();
    BlinkinPattern[] rgbArray;

    public RGBController( HardwareMap hw) {
        super(hw);
        rgbArray = new BlinkinPattern[]{BlinkinPattern.YELLOW, BlinkinPattern.GREEN};
    }
    public RGBController( HardwareMap hw, String name ) {
        super(hw, name);
        rgbArray = new BlinkinPattern[]{BlinkinPattern.BLUE, BlinkinPattern.BLUE};
    }

    //Take in two parameters of color and alternate between them every second
    public void setColor(BlinkinPattern color) {
        super.setPattern(color);
    }

    public void writeColors(BlinkinPattern[] colors) {
        secondsPassed.reset();
        rgbArray = colors;
    }

    public void update() {
       super.setPattern(rgbArray[getUpdateRotation()]);
    }

    public BlinkinPattern[] getRgbArray() {
        return rgbArray;
    }

    public ElapsedTime getTimer() {
        return secondsPassed;
    }

    public int getUpdateRotation() {
        return secondsPassed.seconds() % 1 < 0.5 ? 0 : 1;
    }
}
