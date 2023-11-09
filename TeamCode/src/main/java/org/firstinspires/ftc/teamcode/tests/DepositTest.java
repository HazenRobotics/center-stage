package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class DepositTest extends LinearOpMode {
    Deposit deposit;
    GamepadEvents controller1;
    @Override
    public void runOpMode() throws InterruptedException {
        deposit = new Deposit(hardwareMap, "release", "angler");
        controller1 = new GamepadEvents( gamepad1 );

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                deposit.setReleasePosition( Deposit.ReleaseStates.EXTENDED );
            else if (gamepad1.b)
                deposit.setReleasePosition( Deposit.ReleaseStates.RETRACTED );
            else if (gamepad1.y)
                deposit.setReleasePosition( Deposit.ReleaseStates.DROP_ONE );
            else if (controller1.x.onPress())
                deposit.releaseToggle( );

            if (gamepad1.dpad_right)
                deposit.setAnglePosition( Deposit.AngleStates.GRAB );
            else if (gamepad1.dpad_down)
                deposit.setAnglePosition( Deposit.AngleStates.DROP_FLOOR );
            else if (gamepad1.dpad_left)
                deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );
            else if (controller1.dpad_up.onPress())
                deposit.angleToggle( );

            telemetry.addData( "release position", deposit.getReleasePosition() );
            telemetry.addData( "angler position", deposit.getAnglePosition() );
            telemetry.update();
            controller1.update();
        }

    }
}
