package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class IntakeTest extends LinearOpMode {

	KhepriBot robot;
	GamepadEvents controller;
	ElapsedTime timer;
	double intakePower = 0, intakeSpeed = 0.8, normalizedPowerMultiplier;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot(hardwareMap, telemetry);
		controller = new GamepadEvents(gamepad1);
		timer = new ElapsedTime( );

		waitForStart();

		while( opModeIsActive() ) {

			normalizedPowerMultiplier = 12.0 / robot.hubs.get(0).getInputVoltage(VoltageUnit.VOLTS);

			// 0.235 top pixel
			// 0.185 bottom pixel

//			intakePower = normalizedPowerMultiplier *  (((int) timer.seconds() % 2 == 0) ? 0.5 : 0.8) * intakeSpeed;
			intakePower = normalizedPowerMultiplier *  intakeSpeed;

			if (controller.left_bumper.onPress())
				robot.intake.foldIntake();
			else if (controller.right_bumper.onPress())
				robot.intake.deployIntake( 0.8 );

//			if (robot.intake.getDeploymentState() != Intake.DeploymentState.FOLDED)
//				robot.intake.setIntakeMotorPower( intakePower );

			if(controller.dpad_up.onPress())
				intakeSpeed += 0.05;
			if(controller.dpad_down.onPress())
				intakeSpeed -= 0.05;

			if(controller.left_bumper.onRelease())
				timer.reset();

			telemetry.addData( "timer", timer.seconds() );
			telemetry.addData( "onPress", controller.left_bumper.onPress() );
			telemetry.addData( "onRelease", controller.left_bumper.onRelease() );
			telemetry.addData( "intakeSpeed", intakeSpeed );
			telemetry.update();
			controller.update();
		}
	}
}
