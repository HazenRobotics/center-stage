package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BreakBeam;

import java.util.ArrayList;

@TeleOp
public class BreakBeamTest extends LinearOpMode {
    BreakBeam breakBeam;

    ArrayList<BreakBeam> list;
    @Override
    public void runOpMode() throws InterruptedException {
        list = new ArrayList<>();

        list.add( new BreakBeam( hardwareMap, "bb1") );
        list.add( new BreakBeam( hardwareMap, "bb2") );
        list.add( new BreakBeam( hardwareMap, "bb3") );
        list.add( new BreakBeam( hardwareMap, "bb4") );
        list.add( new BreakBeam( hardwareMap, "bb5") );
        list.add( new BreakBeam( hardwareMap, "bb6") );

        waitForStart();

        while (opModeIsActive()) {
            for( int i = 0; i < list.size(); i++ ) telemetry.addData( "bb" + i, list.get( i ) );
            telemetry.update();
        }
    }
}
