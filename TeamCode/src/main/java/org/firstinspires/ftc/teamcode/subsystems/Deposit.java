package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    Servo angler, release;

    public enum ReleaseStates {
        RETRACTED(0.241),
        EXTENDED(0.4),
        DROP_ONE(0.296),
        HOLD_ONE(0.33);
        private final double position;
        ReleaseStates (double pos) {
            position = pos;
        }
        double getPosition() {
            return position;
        }
    }

    public enum AngleStates {
        GRAB( 0.265 ),
        DROP_FLOOR( 0.363 ),
        DROP_FLOOR_AUTO( 0.42 ),
        DROP_BACKDROP( 0.583 ),
        FIX_BUCKET(0.22);
        private final double position;

        AngleStates( double pos ) {
            position = pos;
        }

        double getPosition( ) {
            return position;
        }
    }

    AngleStates angleState = AngleStates.GRAB;
    ReleaseStates releaseState = ReleaseStates.RETRACTED;

    public Deposit( HardwareMap hw ) {
        this(hw, "release", "pivot");
    }
    public Deposit( HardwareMap hw, String releaseName, String anglerName) {
        release = hw.get(Servo.class, releaseName);
        angler = hw.get(Servo.class, anglerName);
    }

    /**
     * Shifts outtake state to next state; none -> 2 pixel loaded -> 1 pixel loaded -> none
     * Repeats. Not able to go from 2 to none.
     */
    public void releaseToggle() {
        switch( releaseState ) {
            case RETRACTED:
                setReleasePosition( ReleaseStates.EXTENDED );
                break;
            case EXTENDED:
                setReleasePosition( ReleaseStates.DROP_ONE );
                break;
            case DROP_ONE:
                setReleasePosition( ReleaseStates.RETRACTED );
                break;
        }
    }

    public void setReleasePosition (ReleaseStates state) {
        releaseState = state;
        release.setPosition(releaseState.getPosition());
    }
    public void setAnglePosition (AngleStates state) {
        angleState = state;
        angler.setPosition(angleState.getPosition());
    }

    public double getReleasePosition() {
        return release.getPosition();
    }
    public double getAnglePosition() {
        return angler.getPosition();
    }
}
