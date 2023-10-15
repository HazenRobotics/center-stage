package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlywheelLauncher {
    DcMotorEx[] flywheels = new DcMotorEx[2];
    Servo servo;

    public enum ReleaseStates {
        RETRACTED(0.4), EXTENDED(0.74);

        double position;

        ReleaseStates(double pos) {
            position = pos;
        }

        double getPosition() {
            return position;
        }
    }

    ReleaseStates releaseState = ReleaseStates.RETRACTED;


    public FlywheelLauncher(HardwareMap hw) {
        this(hw, new String[]{"left", "right"}, new boolean[]{true, false}, "servo" );
    }

    public FlywheelLauncher ( HardwareMap hw, String[] flywheelNames, boolean[] flywheelsReversed,
                              String servoName ) {
        for( int i = 0; i < flywheels.length; i++ ) {
            flywheels[i] = hw.get( DcMotorEx.class, flywheelNames[i] );
            flywheels[i].setDirection( flywheelsReversed[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
            flywheels[i].setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
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
