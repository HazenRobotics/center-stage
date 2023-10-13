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

public class IntakeColorSensor {
    Telemetry telemetry;
    ColorSensor cs;
    Field.Pixel pixelColor;

    float[] hsv = new float[3];

    public IntakeColorSensor(HardwareMap hardwareMap, Telemetry t, String ColorSensorName) {
        cs = hardwareMap.get(ColorSensor.class, ColorSensorName);
        telemetry = t;
    }

    public void readPixelColor() {
        int red = Range.clip( cs.red( ) / 32, 0, 255 );
        int green = Range.clip( cs.green( ) / 32, 0, 255 );
        int blue = Range.clip( cs.blue( ) / 32, 0, 255 );

        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        float saturation = hsv[1];

        //HSV Thresholds:
        //Purple: 200 - 215
        //White: 160 - 181
        //Green: 115 - 144
        //Yellow: 60 - 98

        //cs1 defaults to 220/230, cs2 defaults to 240

        //note here: if the pixel is too close to the sensor, it will output (0.0, 0.0, 1.0)

        if(hue < 98) pixelColor = YELLOW;
        else if(hue < 144) pixelColor = GREEN;
        else if(hue < 181) pixelColor = WHITE;
        else if(hue < 215) pixelColor = PURPLE;
        else pixelColor = NONE;
    }

    public Field.Pixel getPixelColor() {
        return pixelColor;
    }
    public float[] getHSV() {
        return hsv;
    }

    public void getTelemetry() {
        //update pixel color, then get pixel color for telemetry
        readPixelColor();
        telemetry.addData("Color: ", getPixelColor());
        telemetry.addData("HSV: ", Arrays.toString(getHSV()));
    }

}
