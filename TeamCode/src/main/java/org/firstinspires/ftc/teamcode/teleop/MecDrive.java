package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;


@TeleOp (name = "Nerds")

public class MecDrive extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    Claw claw;
    Lift lift;
    GamepadEvents controller1;
//

    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        claw = new Claw(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        controller1 = new GamepadEvents(gamepad1);
    }

    @Override
    public void loop() {

         if(controller1.a.onPress())
             claw.openClaw();
         if(controller1.b.onPress())
             claw.closeClaw();

         lift.armUp(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());


        double drive = -controller1.right_stick_y;
        double strafe = controller1.right_stick_x;
        double rotate = controller1.left_stick_x;

        frontLeft.setPower(drive + rotate + strafe);
        backLeft.setPower(drive + rotate - strafe);
        frontRight.setPower(drive - rotate - strafe);
        backRight.setPower(drive - rotate + strafe);


        telemetry.addData("fl: ", frontLeft.getPower());
        telemetry.addData("bl: ", backLeft.getPower());
        telemetry.addData("fr: ", frontRight.getPower());
        telemetry.addData("br: ", backRight.getPower());
        telemetry.addData("drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("rotate: ", rotate);


    }


}

