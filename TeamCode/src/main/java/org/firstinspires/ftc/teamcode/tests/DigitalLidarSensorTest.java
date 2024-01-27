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

    ArrayList<DigitalLidarSensor> list = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        list.add( new DigitalLidarSensor( hardwareMap, "sensor1" ) );
        list.add( new DigitalLidarSensor( hardwareMap, "sensor2" ) );
        list.add( new DigitalLidarSensor( hardwareMap, "sensor3" ) );
        list.add( new DigitalLidarSensor( hardwareMap, "sensor4" ) );
        list.add( new DigitalLidarSensor( hardwareMap, "sensor5" ) );
        list.add( new DigitalLidarSensor( hardwareMap, "sensor6" ) );

        waitForStart();

        while (opModeIsActive()) {
            for( int i = 0; i < list.size(); i++ ) telemetry.addData( "" + i, "" + list.get( i ).isBlocked() );
            telemetry.update();
        }
    }
}
