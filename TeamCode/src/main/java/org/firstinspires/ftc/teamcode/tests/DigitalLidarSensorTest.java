package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BreakBeam;
import org.firstinspires.ftc.teamcode.subsystems.DigitalLidarSensor;

import java.util.ArrayList;

@TeleOp
//@Disabled
public class DigitalLidarSensorTest extends LinearOpMode {

    DigitalLidarSensor sensor1, sensor2;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor1 = new DigitalLidarSensor( hardwareMap, "sensor1" );
        sensor2 = new DigitalLidarSensor( hardwareMap, "sensor2" );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData( "1 blocked?", sensor1.isBlocked() );
            telemetry.addData( "2 blocked?", sensor2.isBlocked() );
            telemetry.update();
        }
    }
}
