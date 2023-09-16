package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Array;

public class Intake {
    DcMotor intakeMotor;
    IntakeState intakeState = IntakeState.NONE; //default state, nothing inside

    public enum IntakeState {
        ONESLOT,
        TWOSLOT,
        OVERFLOW,
        NONE;
    }
    public Intake ( HardwareMap hw, String name ) {
        intakeMotor = hw.get(DcMotor.class, name);
    }

    public void run( IntakeState state ) {
        //run intake motors here



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

}
