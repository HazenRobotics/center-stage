package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Field;

public class Intake {
    Telemetry telemetry;
    DcMotor intakeMotor; //intake motor

//    boolean[] sensorDetectArray = {false, false};
    IntakeState intakeState = IntakeState.NONE; //default state, nothing inside
    IntakeColorSensor cs1; //first color sensor
    IntakeColorSensor cs2; //second color sensor
    IntakeBreakBeamSensor breakBeam; //break beam sensor
    Field.Pixel[] pixelColorArray = new Field.Pixel[2]; //array for pixel colors

    public enum IntakeState {
        ONESLOT,
        TWOSLOT,
        OVERFLOW,
        NONE
    }
    public Intake ( HardwareMap hw, Telemetry t, String motorName, String firstColorSensorName, String secondColorSensorName, String breakBeamSensorName) {
        intakeMotor = hw.get(DcMotor.class, motorName);
        cs1 = new IntakeColorSensor(hw, t, firstColorSensorName);
        cs2 = new IntakeColorSensor(hw, t, secondColorSensorName);
        breakBeam = new IntakeBreakBeamSensor(hw, t, breakBeamSensorName);
        telemetry = t;
    }

    public Field.Pixel[] getPixelColorArray() {
        return pixelColorArray;
    }

    //updates pixel color array
    public void updatePixelColorArray() {
        cs1.readPixelColor();
        cs2.readPixelColor();
        pixelColorArray[0] = cs1.getPixelColor();
        pixelColorArray[1] = cs2.getPixelColor();
    }

    //returns intake state
    public IntakeState getIntakeState() {
        return intakeState;
    }

    //put this in the teleop... not here

//    public void loop( IntakeState state ) {
//        switch ( state ) {
//            case OVERFLOW:
//                //reverse motors/dispose of pixels until !OVERFLOW
//                break;
//            case TWOSLOT:
//                break;
//            case ONESLOT:
//                //desire second pixel
//                break;
//            default:
//                //desire one pixel
//                break;
//        }
//
//    }


    //retrieves telemetry from color sensor and break beam class, adds intake state and pixel slots.
    public void getTelemetry() {
        cs1.getTelemetry();
        cs2.getTelemetry();
        breakBeam.getTelemetry();
        telemetry.addData("Intake: ", getIntakeState());
        telemetry.addData("Pixel Slot 1: ", pixelColorArray[0]);
        telemetry.addData("Pixel Slot 2: ", pixelColorArray[1]);
    }

}
