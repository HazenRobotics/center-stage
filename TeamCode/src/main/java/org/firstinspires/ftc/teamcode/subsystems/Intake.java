package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Field;

public class Intake {
    Telemetry telemetry;
    DcMotor intakeMotor;
    ColorSensor firstColorSensor;
    ColorSensor secondColorSensor;
    //anything in the sensors, by chance?
    boolean[] sensorDetectArray = {false, false};

    /*color thresholds for identifying different pixel colors;
    might convert first to HSV or other color format, because
    RGB doesn't seem to be accurate enough.
     */

    //the following holds placeholder zeroes, as we do not currently know testable values
    double[] whiteThreshold = {0, 0, 0};
    double[] greenThreshold = {0, 0, 0};
    double[] purpleThreshold = {0, 0, 0};
    double[] yellowThreshold = {0, 0, 0};


    IntakeState intakeState = IntakeState.NONE; //default state, nothing inside
    IntakeColorSensor cs1;
    IntakeColorSensor cs2;
    Field.Pixel[] pixelColorArray = new Field.Pixel[2];

    public enum IntakeState {
        ONESLOT,
        TWOSLOT,
        OVERFLOW,
        NONE
    }
    public Intake ( HardwareMap hw, Telemetry t, String motorName, String firstColorSensorName, String secondColorSensorName) {
        intakeMotor = hw.get(DcMotor.class, motorName);
        cs1 = new IntakeColorSensor(hw, t, firstColorSensorName);
        cs2 = new IntakeColorSensor(hw, t, secondColorSensorName);
        telemetry = t;
    }

    public Field.Pixel[] getPixelColorArray() {
        return pixelColorArray;
    }

    //takes in sensor number, inputs the color into the array
    public void setPixelColorArray(int sensorNum, Field.Pixel pixelColor) {
        pixelColorArray[sensorNum + 1] = pixelColor;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void loop( IntakeState state ) {
        switch ( state ) {
            case OVERFLOW:
                //reverse motors/dispose of pixels until !OVERFLOW
                break;
            case TWOSLOT:
                break;
            case ONESLOT:
                //desire second pixel
                break;
            default:
                //desire one pixel
                break;
        }

    }

    public void addTelemetry() {
        telemetry.addData("Intake: ", getIntakeState());
        telemetry.addData("Pixel Slot 1: ", pixelColorArray[0]);
        telemetry.addData("Pixel Slot 2: ", pixelColorArray[1]);
    }

}
