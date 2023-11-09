package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Field.Pixel.GREEN;
import static org.firstinspires.ftc.teamcode.utils.Field.Pixel.NONE;
import static org.firstinspires.ftc.teamcode.utils.Field.Pixel.PURPLE;
import static org.firstinspires.ftc.teamcode.utils.Field.Pixel.WHITE;
import static org.firstinspires.ftc.teamcode.utils.Field.Pixel.YELLOW;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Field;

import java.util.Arrays;

public class IntakeColourSensor {
    Telemetry telemetry;
    ColorSensor cs;
    Field.Pixel pixelColour;

    float[] hsv = new float[3];

    public IntakeColourSensor(HardwareMap hardwareMap, Telemetry t, String ColourSensorName) {
        cs = hardwareMap.get(ColorSensor.class, ColourSensorName);
        telemetry = t;
    }

    public void readPixelColour() {
        int red = Range.clip( cs.red( ) / 32, 0, 255 );
        int green = Range.clip( cs.green( ) / 32, 0, 255 );
        int blue = Range.clip( cs.blue( ) / 32, 0, 255 );

        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        float saturation = hsv[1];

        //HSV Thresholds @house
        //Purple: 200 - 215
        //White: 160 - 181
        //Green: 115 - 144
        //Yellow: 60 - 98

        //HSV Thresholds @hazen
        //Purple: 187 - 212
        //White: 160 - 180
        //Green: 110 - 133
        //Yellow: 60 - 90

        //cs1 defaults to 220/230, cs2 defaults to 240 @victor's house
        //cs1 defaults to 210, cs2 defaults to 220 @hazen

        //note here: if the pixel is too close to the sensor, it will output (0.0, 0.0, 1.0)\

        if(hue < 90) pixelColour = YELLOW;
        else if(hue < 133) pixelColour = GREEN;
        else if(hue < 180) pixelColour = WHITE;
        else if(hue < 212) pixelColour = PURPLE;
        else pixelColour = NONE;

//        if(hue < 98) pixelColour = YELLOW;
//        else if(hue < 144) pixelColour = GREEN;
//        else if(hue < 181) pixelColour = WHITE;
//        else if(hue < 215) pixelColour = PURPLE;
//        else pixelColour = NONE;
    }

    public Field.Pixel getPixelColour() {
        return pixelColour;
    }
    public float[] getHSV() {
        return hsv;
    }

    public void addTelemetry() {
        //update pixel color, then get pixel color for telemetry
        readPixelColour();
        telemetry.addData("Colour: ", getPixelColour());
        telemetry.addData("HSV: ", Arrays.toString(getHSV()));
    }

}
