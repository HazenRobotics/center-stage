package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Array;

public class Intake {
    DcMotor intakeMotor;
    static IntakeState intakeState = IntakeState.NONE; //default state, nothing inside

    public enum IntakeState {
        ONESLOT(1),
        TWOSLOT(2),
        OVERFLOW(-1),
        NONE(0);
        IntakeState() {}
        private int key;
        private IntakeState (int key) { this.key = key;}
        static int getIntakeStateKey () { return intakeState.key; }
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
