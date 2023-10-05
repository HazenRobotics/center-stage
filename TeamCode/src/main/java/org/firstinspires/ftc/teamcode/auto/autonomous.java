package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class autonomous extends LinearOpMode {
    DcMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        //stuff before init
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        //stuff after pressing start

        move(0.5, 0, 0);
        sleep(2000);
        move(0, 0, 1);
        sleep(5000);
        move(0, 0.2, 0);
        sleep(1000);
        move(0, 0, 0.3);
        sleep(3000);
        move(0, 0, 0.6);
        sleep(3000);
        move(0, 0, 0.9);
        sleep(3000);



    }

    public void move(double drive, double strafe, double rotate) {
        frontLeft.setPower(drive + rotate + strafe);
        backLeft.setPower(drive + rotate - strafe);
        frontRight.setPower(drive - rotate - strafe);
        backRight.setPower(drive - rotate + strafe);
    }//




}
