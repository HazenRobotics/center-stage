package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.IntakeBreakBeamSensor;

@TeleOp
public class BreakBeamTest extends LinearOpMode {
    IntakeBreakBeamSensor breakBeam;

    @Override
    public void runOpMode() throws InterruptedException {
        breakBeam = new IntakeBreakBeamSensor( hardwareMap, telemetry, "breakBeam");
        waitForStart();

        while (opModeIsActive()) {
            breakBeam.addTelemetry();
            telemetry.update();
        }
    }
}
