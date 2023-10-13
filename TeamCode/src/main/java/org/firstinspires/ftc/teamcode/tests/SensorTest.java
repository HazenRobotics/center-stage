package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class SensorTest extends LinearOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap,
                telemetry,
                "intakeMotor",
                "color1",
                "color2",
                "breakBeam");

        waitForStart();
        while(opModeIsActive()) {
            intake.getTelemetry();
            telemetry.update();
        }
    }
}
