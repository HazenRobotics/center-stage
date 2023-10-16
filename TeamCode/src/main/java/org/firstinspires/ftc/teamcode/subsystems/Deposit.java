package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    Servo angler, release;

    public enum ReleaseStates {
        RETRACTED(0.582, 0), EXTENDED(0.816, 1), DROP_ONE(0.636, 2);
        private final double position;
        private final int index;
        private ReleaseStates (double pos, int ind) {
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

    ReleaseStates[] states;
    int stateIndex;
    public Deposit( HardwareMap hw, String releaseName, String anglerName) {
        release = hw.get(Servo.class, releaseName);
        angler = hw.get(Servo.class, anglerName);
        states = ReleaseStates.values();
        stateIndex = 0;
    }

    /**
     * Shifts outtake state to next state; none -> 2 pixel loaded -> 1 pixel loaded -> none
     * Repeats. Not able to go from 2 to none.
     */
    public void toggle() {
        stateIndex++;
        stateIndex %= 3;
        release.setPosition(states[stateIndex].getPosition());
    }

    public void setReleasePosition (ReleaseStates state) {
        release.setPosition(state.getPosition());
        stateIndex = state.getIndex();
    }

    public double getReleasePosition() {
        return release.getPosition();
    }
}
