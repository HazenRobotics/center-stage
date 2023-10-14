package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelLauncher {

//    can either be CR servos or motors, but is likely going to have to be a motor to control velocity
//    and because CR servos have
//    CRServo left, right;
    DcMotorEx left, right;

    public FlywheelLauncher(HardwareMap hw) {
        this(hw, "left", "right", true, false);
    }

    public FlywheelLauncher (HardwareMap hw, String leftName, String rightName,
                             boolean leftReverse, boolean rightReverse) {
//        left = hw.get( CRServo.class, leftName );
//        right = hw.get( CRServo.class, rightName );
        left = hw.get( DcMotorEx.class, leftName );
        right = hw.get( DcMotorEx.class, rightName );

        left.setDirection( leftReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
        right.setDirection( rightReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
    }

    public void setPower( double power ) {
        left.setPower(power);
        right.setPower(power);
    }

}
