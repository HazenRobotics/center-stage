package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    Outtake outtake;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap, "release", "angler");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                outtake.setReleasePosition( Outtake.ReleaseStates.EXTENDED );
            else if (gamepad1.b)
                outtake.setReleasePosition( Outtake.ReleaseStates.RETRACTED );
            else if (gamepad1.y) {
                outtake.setReleasePosition( Outtake.ReleaseStates.DROP_ONE );
            }

            telemetry.addData( "release position", outtake.getReleasePosition() );
            telemetry.update();
        }

    }
}
