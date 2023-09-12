package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Array;

public class Intake {
    DcMotor intakeMotor;
    ColorSensor firstColorSensor;
    ColorSensor secondColorSensor;
    boolean[] sensorDetectArray = {false, false};

    double[] whiteThreshold = [220, 220, 220];
    double[] greenThreshold = [R, G, B];
    double[] purpleThreshold = [R, G, B];
    double[] yellowThreshold = [R, G, B];

    //consider using weight sensors?

    IntakeState intakeState = IntakeState.NONE; //default state, nothing inside

    public enum IntakeState {
        ONESLOT,
        TWOSLOT,
        OVERFLOW,
        NONE;
    }
    public Intake ( HardwareMap hw, String name ) {
        intakeMotor = hw.get(DcMotor.class, name);
        firstColorSensor = hw.get(ColorSensor.class, name);
        secondColorSensor = hw.get(ColorSensor.class, name);
    }

    public void loop( IntakeState state ) {
        switch ( state ) {
            case OVERFLOW:
                //reverse motors/dispose of pixels until !OVERFLOW
                //telemetry overflow
                break;
            case TWOSLOT:
                //telemetry two
                break;
            case ONESLOT:
                //desire second pixel
                //telemetry one
                break;
            default:
                //desire one pixel
                //telemetry none
                break;
        }
    }

    public boolean[] firstColorSensorResponse ( ) {
        double redPercent = firstColorSensor.red()/255;
        double greenPercent = firstColorSensor.green()/255;
        double bluePercent = firstColorSensor.blue()/255;

        if(greenPercent > ) {

        }
    }
}
