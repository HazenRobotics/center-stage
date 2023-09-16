package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Detector {
    ColorSensor firstColorSensor;
    float[] rgb1 = new float[3];
    ColorSensor secondColorSensor;
    float[] rgb2 = new float[3];
    double whiteThresh, greenThresh, purpleThresh, yellowThresh;
    boolean[] sensorDetectArray = {false, false};

    //RGB Values
//    double[] whiteVal = {255.0, 255.0, 255.0};
//    double[] greenVal = {0.0, 255.0, 0.0};
//    double[] purpleVal = {255.0, 0.0, 255.0};
//    double[] yellowVal = {255.0, 255.0, 0.0};

//    //HSV Values
//    double[] whiteValHSV = {49-60, any, any};
//    double[] greenValHSV = {85-140, any, any};
//    double[] purpleValHSV = {255.0, 0.0, 255.0};
//    double[] yellowValHSV = {255.0, 255.0, 0.0};

    public boolean isWhite( ) {
        return true;
    }

    public boolean isGreen() {
        return true;
    }

    public boolean isYellow() {
        return true;
    }

    public boolean isPurple() {
        return true;
    }

    public float[] tenToHSV(int red, int green, int blue) {
        float[] hsv = new float[3];
        Color.RGBToHSV(red / 4, green / 4, blue / 4, hsv);
        return hsv;
    }

    public Detector(HardwareMap hw, String colorSens1, String colorSens2) {
        firstColorSensor = hw.get(ColorSensor.class, colorSens1);
        secondColorSensor = hw.get(ColorSensor.class, colorSens2);
    }

    public Field.Pixel[] getColorResponse( ColorSensor c1, ColorSensor c2 ) {
        Field.Pixel[] response = new Field.Pixel[2];
        if(isGreen()) {
            response[0] = Field.Pixel.GREEN;
        }
        else if(isPurple()) {

        }
        else if(isYellow()) {

        }
        else if(isWhite()) {

        }
        else {
            return
        }
        return response;
    }

}

