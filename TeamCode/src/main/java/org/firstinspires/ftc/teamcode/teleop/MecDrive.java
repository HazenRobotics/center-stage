package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;


@TeleOp

public class MecDrive extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, lift;
    Claw claw;

    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        lift = hardwareMap.dcMotor.get("lift");

       claw = new Claw();


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {


         if(gamepad1.dpad_up) {
             claw.openClaw();
         } else if(gamepad1.dpad_down)
         {
             claw.closeClaw();
         }
        double drive = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        frontLeft.setPower(drive + rotate + strafe);
        backLeft.setPower(drive + rotate - strafe);
        frontRight.setPower(drive - rotate - strafe);
        backRight.setPower(drive - rotate + strafe);

        if(gamepad1.a) { frontLeft.setPower(1);}
        if(gamepad1.b) { backLeft.setPower(1);}
        if(gamepad1.x) { frontRight.setPower(1);}
        if(gamepad1.y) { backRight.setPower(1);}


        telemetry.addData("fl: ", frontLeft.getPower());
        telemetry.addData("bl: ", backLeft.getPower());
        telemetry.addData("fr: ", frontRight.getPower());
        telemetry.addData("br: ", backRight.getPower());
        telemetry.addData("drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("rotate: ", rotate);


    }


}

