package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RingLauncherPRBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "PRBotTest",group = "TeleOp")
public class PRBotTeleOp extends OpMode {
    GamepadEvents controller;
    RingLauncherPRBot robot;

    @Override
    public void init( ) {
        robot = new RingLauncherPRBot( hardwareMap );
        robot.drive.setupIMU( hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD );

        controller = new GamepadEvents( gamepad1 );
    }

    @Override
    public void loop( ) {
        double drive = -controller.left_stick_y;
        double strafe = controller.left_stick_x;
        double rotate = controller.right_stick_x;

        robot.drive.fieldCentricDrive( drive, strafe, rotate );

        double flyWheelSpeed = gamepad1.right_trigger;
        robot.launcher.setPower( flyWheelSpeed );

        if( controller.a.onPress() )
            robot.launcher.toggle();

        if( controller.start.onPress() )
            robot.drive.resetIMU();

        controller.update();
    }
}
