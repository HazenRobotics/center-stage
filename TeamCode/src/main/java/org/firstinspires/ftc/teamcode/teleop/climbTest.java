package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class climbTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx climb = hardwareMap.get(DcMotorEx.class,"climb");
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while(opModeIsActive()) {
            climb.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            telemetry.addData("Climb Power",climb.getPower());
            telemetry.addData("Encoder Pos",climb.getCurrentPosition());
            telemetry.update();
        }
    }
}
