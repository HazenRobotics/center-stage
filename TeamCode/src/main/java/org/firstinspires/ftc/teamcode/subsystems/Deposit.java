package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    Servo angler, release;

    public enum ReleaseStates {
        RETRACTED(0.582, 0), EXTENDED(0.816, 1), DROP_ONE(0.636, 2);
        private final double position;
        private final int index;
        ReleaseStates (double pos, int ind) {
            position = pos;
            index = ind;
        }
        double getPosition() {
            return position;
        }
        int getIndex() {
            return index;
        }
    }

    public enum AngleStates {
        GRAB(0 ), DROP_VERT(0.15 ), DROP_ANGLE(0.51 );
        private final double position;
        AngleStates (double pos) {
            position = pos;
        }
        double getPosition() {
            return position;
        }
    }

    ReleaseStates[] releaseStates;
    int releaseStateIndex;

    AngleStates angleState = AngleStates.GRAB;

    public Deposit( HardwareMap hw, String releaseName, String anglerName) {
        release = hw.get(Servo.class, releaseName);
        angler = hw.get(Servo.class, anglerName);
        releaseStates = ReleaseStates.values();
        releaseStateIndex = 0;
    }

    /**
     * Shifts outtake state to next state; none -> 2 pixel loaded -> 1 pixel loaded -> none
     * Repeats. Not able to go from 2 to none.
     */
    public void releaseToggle() {
        releaseStateIndex++;
        releaseStateIndex %= 3;
        release.setPosition( releaseStates[releaseStateIndex].getPosition());
    }

    public void angleToggle() {
        switch( angleState ) {
            case GRAB:
            case DROP_VERT:
                angleState = AngleStates.DROP_ANGLE;
                break;
            case DROP_ANGLE:
                angleState = AngleStates.GRAB;
                break;
        }
        setAnglePosition( angleState );
    }

    public void setReleasePosition (ReleaseStates state) {
        release.setPosition(state.getPosition());
        releaseStateIndex = state.getIndex();
    }
    public void setAnglePosition (AngleStates state) {
        angler.setPosition(state.getPosition());
    }

    public double getReleasePosition() {
        return release.getPosition();
    }
    public double getAnglePosition() {
        return angler.getPosition();
    }
}
