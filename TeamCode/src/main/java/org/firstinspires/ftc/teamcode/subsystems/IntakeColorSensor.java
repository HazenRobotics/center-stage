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

public class IntakeColorSensor {
    Telemetry telemetry;
    ColorSensor cs;
    Field.Pixel pixelColor;
    float[] hsv = new float[3];

    //HSV Thresholds yo
    //190-205 purple
    //84-96 yellow
    //120-132 green
    //150-160 white

    public IntakeColorSensor (HardwareMap hardwareMap, Telemetry t, String colorSensorName) {
        cs = hardwareMap.get(ColorSensor.class, colorSensorName);
        telemetry = t;
    }

    public void readPixelColor() {
        int red = Range.clip( cs.red( ) / 32, 0, 255 );
        int green = Range.clip( cs.green( ) / 32, 0, 255 );
        int blue = Range.clip( cs.blue( ) / 32, 0, 255 );

        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        float saturation = hsv[1];
        if(saturation < 0.4) pixelColor = WHITE;
        else if(hue > 175) pixelColor = PURPLE;
        else if(hue > 115) pixelColor = GREEN;
        else if(hue > 84) pixelColor = YELLOW;
        else pixelColor = NONE;
    }

    public Field.Pixel getPixelColor() {
        return pixelColor;
    }
    public float[] getHSV() {
        return hsv;
    }

}
