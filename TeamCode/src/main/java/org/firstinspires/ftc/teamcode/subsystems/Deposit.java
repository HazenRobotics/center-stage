package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingServo;

@Config

public class Deposit {
    Servo angler, release;
    public static double dropBackDrop = 0.583;

    public enum ReleaseStates {
        RETRACTED(0.31),
        EXTENDED(0.547),
        DROP_ONE(0.375),
        HOLD_ONE(0.5);
        private final double position;
        ReleaseStates (double pos) {
            position = pos;
        }
        double getPosition() {
            return position;
        }
    }

    public enum AngleStates {
        GRAB( 0.423 ),
        DROP_BACKDROP( 0.741 ),
        FIX_BUCKET(0.37),
        STRAIGHT_DOWN( 0.529 );
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
        release = new CachingServo( hw.get(Servo.class, releaseName) );
        angler = new CachingServo( hw.get(Servo.class, anglerName) );
    }

    /**
     * Shifts outtake state to next state; none -> 2 pixel loaded -> 1 pixel loaded -> none
     * Repeats. Not able to go from 2 to none.
     */
    public void releaseToggle() {
        switch( releaseState ) {
            case RETRACTED:
                setReleaseState( ReleaseStates.EXTENDED );
                break;
            case EXTENDED:
                setReleaseState( ReleaseStates.DROP_ONE );
                break;
            case DROP_ONE:
                setReleaseState( ReleaseStates.RETRACTED );
                break;
        }
    }

    public void setReleaseState( ReleaseStates state) {
        releaseState = state;
        setReleasePosition(releaseState.getPosition());
    }
    public void setReleasePosition (double position) {
        release.setPosition(position);
    }
    public void setAngleState( AngleStates state) {
        angleState = state;
        setAnglePosition(angleState.getPosition());
    }

    public AngleStates getAngleState( ) {
        return angleState;
    }

    public ReleaseStates getReleaseState( ) {
        return releaseState;
    }

    public void setAnglePosition ( double position) {
        angler.setPosition(position);
    }

    public double getReleasePosition() {
        return release.getPosition();
    }
    public double getAnglePosition() {
        return angler.getPosition();
    }
}
