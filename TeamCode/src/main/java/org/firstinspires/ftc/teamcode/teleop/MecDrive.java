package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.lang.reflect.Array;
import java.util.ArrayList;


@TeleOp (name = "Nerds")

public class MecDrive extends LinearOpMode
{

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor motors[] = {frontLeft, frontRight, backLeft, backRight};


    Claw claw;
    Lift lift;
    GamepadEvents controller1;
//

    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);

        String  motorNames[] = new String[]{"frontLeft", "frontRight", "backLeft", "backRight"};
        // i represents count
        for(int i = 0; i < motorNames.length; i++) {motors[i] = hardwareMap.get(DcMotor.class, motorNames[i]);}



        claw = new Claw(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a)
                claw.openClaw();
            if (gamepad1.b)
                claw.closeClaw();

            lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


            double drive = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

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
            telemetry.update();
        }

    }


}

