package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Detector {
    ColorSensor colorSensor;

    //    //HSV Values - H values control the color, besides white; organized {min, max}
    //white HSV Values {0, 0, 100} -- CHECK IF WHITE FIRST!! else, ...
    int[] whiteValHSVLow = {0, 0, 90};
    int[] whiteValHSVHigh = {5, 5, 100};
    int[] greenValHSV = {85, 140};
    int[] purpleValHSV = {281, 320};
    int[] yellowValHSV = {51, 60};

    public Detector(HardwareMap hw, String colorSens) {
        colorSensor = hw.get(ColorSensor.class, colorSens);
    }

    //checks if ALL H, S, AND V are all in bound for white. All other colors only need H checked.
    public boolean isWhite(ColorSensor sniffer) {
        return (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] > whiteValHSVLow[0]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] < whiteValHSVHigh[0]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[1] > whiteValHSVLow[1]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[1] < whiteValHSVHigh[1]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[2] > whiteValHSVLow[2]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[2] < whiteValHSVHigh[2];
    }

    public boolean isGreen(ColorSensor sniffer) {
        return (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] > greenValHSV[0]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] < greenValHSV[1];
    }

    public boolean isPurple(ColorSensor sniffer) {
        return (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] > purpleValHSV[0]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] < purpleValHSV[1];
    }

    public boolean isYellow(ColorSensor sniffer) {
        return (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] > yellowValHSV[0]
                && (tenToHSV(sniffer.red(), sniffer.green(), sniffer.blue()))[0] < yellowValHSV[1];
    }

    public float[] tenToHSV(int red, int green, int blue) {
        float[] hsv = new float[3];
        Color.RGBToHSV(red / 4, green / 4, blue / 4, hsv);
        return hsv;
    }

    public Field.Pixel getColorResponse(ColorSensor c1) {
        if (isWhite(c1)) {
            return Field.Pixel.WHITE;
        } else if (isPurple(c1)) {
            return Field.Pixel.PURPLE;
        } else if (isYellow(c1)) {
            return Field.Pixel.YELLOW;
        } else if (isGreen(c1)) {
            return Field.Pixel.GREEN;
        } else {
            return Field.Pixel.NONE;
        }

    }

}

