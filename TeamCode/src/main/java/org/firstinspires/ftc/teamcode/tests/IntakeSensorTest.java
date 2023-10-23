package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class IntakeSensorTest extends LinearOpMode {
    double loopTime = 0.0;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
//        intake = new Intake(hardwareMap,
//                telemetry,
//                "intakeMotor",
//                "color1",
//                "color2",
//                "breakBeam",
//                "deploy");
//
//        waitForStart();
//        while(opModeIsActive()) {
//            intake.updateIntakeCapacity();
//            intake.addTelemetry();
//            //show how fast 1 loop is
//            double loop = System.nanoTime( );
//            telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
//            loopTime = loop;
//            telemetry.update();
//        }
    }
}
