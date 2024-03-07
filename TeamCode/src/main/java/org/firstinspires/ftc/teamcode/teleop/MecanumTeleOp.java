package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.MecanumBot;
import org.firstinspires.ftc.teamcode.robots.RingLauncherPRBot;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelLauncher;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(group = "TeleOp")
public class MecanumTeleOp extends OpMode {
    GamepadEvents controller;
    MecanumBot robot;
    boolean fieldCentric = false;

    @Override
    public void init( ) {
        robot = new MecanumBot( hardwareMap );
        robot.drive.setupIMU( hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT );

        controller = new GamepadEvents( gamepad1 );
    }

    @Override
    public void loop( ) {
        double drive = controller.left_stick_y * (gamepad1.left_stick_button ? 1 : 0.6);
        double strafe = controller.left_stick_x * (gamepad1.left_stick_button ? 1 : 0.6);
        double rotate = -controller.right_stick_x * (gamepad1.right_stick_button ? 1 : 0.6);

        if (fieldCentric) robot.drive.fieldCentricDrive( drive, strafe, rotate );
        else robot.drive.robotCentricDrive( drive, strafe, rotate );

        if( controller.start.onPress() ) robot.drive.resetIMU();

        if( controller.dpad_up.onPress() ) fieldCentric = !fieldCentric;

        telemetry.addData( "fieldCentric", fieldCentric );

        controller.update();
        telemetry.update();
    }
}
