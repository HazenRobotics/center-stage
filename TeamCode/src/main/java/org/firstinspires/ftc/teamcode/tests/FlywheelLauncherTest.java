package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelLauncher;

@TeleOp
public class FlywheelLauncherTest extends LinearOpMode {

    FlywheelLauncher launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        launcher = new FlywheelLauncher(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            launcher.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
        }
    }
}
