package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp(group = "Test" )
public class PIDF_Tuning extends OpMode {

	Lift lift;

	public static double p = 0, i = 0, d = 0, f = 0;

	public static double target = 0;

	PIDController controller;

	@Override
	public void init( ) {
		controller = new PIDController( p, i, d );
		lift = new Lift( hardwareMap );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
	}

	@Override
	public void loop( ) {
		controller.setPID( p, i, d );

		double motorPos = lift.getMotorPosition();

		lift.setPower( controller.calculate( motorPos, target ) + f );

		if (gamepad1.x) {
			target = 0;
			lift.resetLift();
		}

		telemetry.addData( "lift pos", motorPos );
		telemetry.addData( "target pos", target );
		telemetry.update();

	}
}
