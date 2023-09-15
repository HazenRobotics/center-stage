package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Detector {
    ColorSensor firstColorSensor;
    ColorSensor secondColorSensor;
    double whiteThresh, greenThresh, purpleThresh, yellowThresh;
    boolean[] sensorDetectArray = {false, false};
    double[] whiteVal = {220.0, 220.0, 220.0};
    double[] greenVal = {R, G, B};
    double[] purpleVal = {R, G, B};
    double[] yellowVal = {(190, 190, 66};

    public boolean isYellow( double R, double G, double B) {
        if( (R >= (0.3174 * B + 169)) && (G >= (0.3174 * B + 169)) && (R <= -B + 255) && (G <= -B + 255) && B <= 60 )
            return true;
        else
            return false;
    }

    public Detector(HardwareMap hw, String name) {
        firstColorSensor = hw.get(ColorSensor.class, name);
        secondColorSensor = hw.get(ColorSensor.class, name);
    }

    public boolean[] firstColorSensorResponse() {
        double redPercent = firstColorSensor.red() / 255;
        double greenPercent = firstColorSensor.green() / 255;
        double bluePercent = firstColorSensor.blue() / 255;

        //yellow
        /* Yellow	#FFFF00	(255,255,0)	(60°,100%,100%)
        red-green: NO blue (tolerance +/- 10), high red, high green (190-255);
        */
        if (greenPercent * > 190 && redPercent > 190 && bluePercent < 10) {

        }
    }
}
