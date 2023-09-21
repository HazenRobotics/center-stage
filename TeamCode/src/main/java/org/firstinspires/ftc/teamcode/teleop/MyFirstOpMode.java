package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MyFirstOpMode extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public void runOpMode() throws InterruptedException {
        init();
        moveForwardAndStop();
    }

    @Override
    public void init() {
        // code before start (Initializing Hardware, Adding telemetry data, Setting motor direction)
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"front_right_motor"));
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class,"back_right_motor"));

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode: ", "Waiting");
        telemetry.update();
    }

    public void moveForwardAndStop() throws InterruptedException {
        telemetry.addData("Mode: ","Running");
        telemetry.update();

        frontLeftMotor.setPower(0.25);
        frontRightMotor.setPower(0.25);
        backLeftMotor.setPower(0.25);
        backRightMotor.setPower(0.25);

        Thread.sleep(2000);

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    @Override
    public void loop() {

    }
}
