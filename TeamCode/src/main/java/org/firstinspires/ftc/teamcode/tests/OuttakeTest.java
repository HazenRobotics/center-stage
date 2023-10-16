package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    Deposit deposit;
    @Override
    public void runOpMode() throws InterruptedException {
        deposit = new Deposit(hardwareMap, "release", "angler");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                deposit.setReleasePosition( Deposit.ReleaseStates.EXTENDED );
            else if (gamepad1.b)
                deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED );
            else if (gamepad1.y)
                deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
            else if (gamepad1.x)
                deposit.toggle( );

            telemetry.addData( "release position", deposit.getReleasePosition() );
            telemetry.update();
        }

    }
}
