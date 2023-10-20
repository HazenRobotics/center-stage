package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecDrive {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    String motorNames[];


    DcMotor motors[] = {frontLeft, frontRight, backLeft, backRight};

    public MecDrive() {
        Lift = new Lift(hardwareMap, telemetry);
         motorNames[] = {"frontLeft"," frontRight", "backLeft", "backRight"};
        for(int i = 0; i < motors.length; i++) { motors[i] = HardwareMap.get(DcMotor.class , motorNames[i]);}

    }

    public void moveBot(double drive, double strafe, double rotate) _{
        frontLeft.setPower(drive + rotate + strafe);
        backLeft.setPower(drive + rotate - strafe);
        frontRight.setPower(drive - rotate + strafe);
        backRight.setPower(drive - rotate + strafe);
    }



}