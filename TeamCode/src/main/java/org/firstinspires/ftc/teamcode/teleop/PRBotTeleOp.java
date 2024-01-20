package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.RingLauncherPRBot;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelLauncher;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "PRBotTest",group = "TeleOp")
public class PRBotTeleOp extends OpMode {
    GamepadEvents controller;
    RingLauncherPRBot robot;
    ElapsedTime timer;
    boolean fieldCentric = false;
    boolean flywheelToggle = false;
    boolean autoFire = false;

    @Override
    public void init( ) {
        robot = new RingLauncherPRBot( hardwareMap );
        robot.drive.setupIMU( hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP );

        controller = new GamepadEvents( gamepad1 );
        timer = new ElapsedTime();
    }

    @Override
    public void loop( ) {
        double drive = -controller.left_stick_y * (gamepad1.left_stick_button ? 1 : 0.6);
        double strafe = controller.left_stick_x * (gamepad1.left_stick_button ? 1 : 0.6);
        double rotate = -controller.right_stick_x * (gamepad1.right_stick_button ? 1 : 0.6);

        if (fieldCentric) robot.drive.fieldCentricDrive( drive, strafe, rotate );
        else robot.drive.robotCentricDrive( drive, strafe, rotate );

        double flyWheelSpeed = gamepad1.right_trigger * 0.7 + (gamepad1.left_trigger * 0.5);
        robot.launcher.setPower( flyWheelSpeed + (flywheelToggle ? 0.7 : 0) );

        if( timer.seconds() > 0.5 ) robot.launcher.setServoPos( FlywheelLauncher.ReleaseStates.RETRACTED );

        if( (controller.a.onPress() || autoFire) && timer.seconds() > 1 ) {
            timer.reset( );
            robot.launcher.toggle( );
        }

        if( controller.start.onPress() ) robot.drive.resetIMU();

        if( controller.x.onPress() ) fieldCentric = !fieldCentric;

        if( controller.y.onPress() ) {
            flywheelToggle = !flywheelToggle;
//            autoFire = flywheelToggle;
        }

        telemetry.addData( "power", robot.launcher.getPower() );

        controller.update();
        telemetry.update();
    }
}
