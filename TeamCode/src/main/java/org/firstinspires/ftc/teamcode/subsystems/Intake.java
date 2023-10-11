package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Field;

public class Intake {
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
    double[] whiteThreshold = [0, 0, 0];
    double[] greenThreshold = [0, 0, 0];
    double[] purpleThreshold = [0, 0, 0];
    double[] yellowThreshold = [0, 0, 0];

    //consider using weight sensors?

    static IntakeState intakeState = IntakeState.NONE; //default state, nothing inside

    public enum IntakeState {
        ONESLOT(1),
        TWOSLOT(2),
        OVERFLOW(-1),
        NONE;
        IntakeState() {}
        private int key;
        private IntakeState (int key) { this.key = key;}
        static int getIntakeStatekey () { return intakeState.key; }
        static IntakeState getIntakeState (int x) {
            if (x == 1) { return ONESLOT; }
            else if (x == 2) { return TWOSLOT; }
            else if (x == -1) { return OVERFLOW; }
            else if (x == 0) { return NONE;}
            else throw new IllegalArgumentException();
        }
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
        telemetry.addData("Intake: ", IntakeState.getIntakeStatekey());
    }

    public Field.Pixel firstColorSensorResponse ( ) {
        double redPercent = firstColorSensor.red()/255;
        double greenPercent = firstColorSensor.green()/255;
        double bluePercent = firstColorSensor.blue()/255;

        if(redPercent > [greenThreshold[0]] && greenPercent > [greenThreshold[1]] && bluePercent > [greenThreshold]) {
            return
        }
    }
}
