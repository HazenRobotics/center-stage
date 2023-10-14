package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.EnumMap;

public class FlywheelLauncher {
    DcMotorEx[] flywheels = new DcMotorEx[2];
    Servo servo;

    public enum ReleaseStates {
        RETRACTED(0.5 ), EXTENDED(1);

        double position;

        ReleaseStates(double pos) {
            position = pos;
        }

        double getPosition() {
            return position;
        }
    }

    ReleaseStates releaseState;

    public FlywheelLauncher(HardwareMap hw) {
        this(hw, new String[]{"left", "right"}, new boolean[]{true, false}, "servo" );
    }

    public FlywheelLauncher ( HardwareMap hw, String[] flywheelNames, boolean[] flywheelsReversed,
                              String servoName ) {
        for( int i = 0; i < flywheels.length; i++ ) {
            flywheels[i] = hw.get( DcMotorEx.class, flywheelNames[i] );
            flywheels[i].setDirection( flywheelsReversed[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
        }

        servo = hw.get( Servo.class, servoName);
    }

    public void setPower( double power ) {
        for( int i = 0; i < flywheels.length; i++ )
            flywheels[i].setPower( power );
    }
    public void setServoPos( ReleaseStates state ) {
        servo.setPosition( state.getPosition() );
        releaseState = state;
    }
    public void toggle() {
        switch( releaseState ) {
            case RETRACTED:
                setServoPos( ReleaseStates.EXTENDED );
                break;
            case EXTENDED:
                setServoPos( ReleaseStates.RETRACTED );
                break;
        }
    }


}
