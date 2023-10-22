package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    Servo angler, release;

    public enum ReleaseStates {
        RETRACTED(0, 0), EXTENDED(0.27, 1), DROP_ONE(0.126, 2);
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
        GRAB(0.03 ), DROP_FLOOR(0.229 ), DROP_BACKDROP( 0.54 );
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
            case DROP_FLOOR:
                angleState = AngleStates.DROP_BACKDROP;
                break;
            case DROP_BACKDROP:
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
