package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BreakBeamSensor;

@TeleOp
public class BreakBeamTest extends LinearOpMode {
    BreakBeamSensor breakBeam;

    @Override
    public void runOpMode() throws InterruptedException {
        breakBeam = new BreakBeamSensor( hardwareMap, telemetry, "breakBeam");
        waitForStart();

        while (opModeIsActive()) {
            breakBeam.addTelemetry();
            telemetry.update();
        }
    }
}
