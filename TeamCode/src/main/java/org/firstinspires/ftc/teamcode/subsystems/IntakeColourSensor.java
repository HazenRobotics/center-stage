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

    public IntakeColourSensor(HardwareMap hardwareMap, Telemetry t, String colourSensorName) {
        cs = hardwareMap.get(ColorSensor.class, colourSensorName);
        telemetry = t;
    }

    public void readPixelColour() {
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

        //note here: if the pixel is too close to the sensor, it will output (0.0, 0.0, 1.0)
        //it is defaulted to WHITE in this code
        if(hue > 200 && hue < 215) pixelColour = PURPLE;
        else if(hue > 160 && hue <= 181 || hue < 2 && saturation < 2) pixelColour = WHITE;
        else if(hue > 115 && hue < 144) pixelColour = GREEN;
        else if(hue > 60 && hue < 98) pixelColour = YELLOW;
        else pixelColour = NONE;
    }

    public Field.Pixel getPixelColour() {
        return pixelColour;
    }
    public float[] getHSV() {
        return hsv;
    }

    public void getTelemetry() {
        //update pixel color, then get pixel color for telemetry
        readPixelColour();
        telemetry.addData("Colour: ", getPixelColour());
        telemetry.addData("HSV: ", Arrays.toString(getHSV()));
    }

}
