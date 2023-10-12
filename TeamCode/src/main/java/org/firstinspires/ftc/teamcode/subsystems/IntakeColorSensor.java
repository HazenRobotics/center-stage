package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Field;

public class IntakeColorSensor {
    ColorSensor cs;
    Field.Pixel pixelColor;
    float[] fsv;

    public IntakeColorSensor (HardwareMap hardwareMap, String colorSensorName) {
        cs = hardwareMap.get(ColorSensor.class, colorSensorName);
    }

    public void readPixelColor() {
        int red = Range.clip( cs.red( ) / 32, 0, 255 );
        int green = Range.clip( cs.green( ) / 32, 0, 255 );
        int blue = Range.clip( cs.blue( ) / 32, 0, 255 );

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        //if(white) pixelColor = WHITE;
        //else if(green) pixelColor = GREEN;
        //else if(purple) pixelColor = PURPLE;
        //else if(yellow) pixelColor = YELLOW;
        //else pixelColor = NONE;
    }

    public Field.Pixel getPixelColor() {
        return pixelColor;
    }

}
