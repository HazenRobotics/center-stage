package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class OuttakeTest extends LinearOpMode {
	Lift lift;
	Deposit deposit;
	GamepadEvents controller1;
	@Override
	public void runOpMode( ) throws InterruptedException {
		lift = new Lift( hardwareMap );
		deposit = new Deposit( hardwareMap, "release", "angler" );
		controller1 = new GamepadEvents( gamepad1 );

		waitForStart();

		while( opModeIsActive() ) {
			lift.setPower( gamepad1.right_trigger - (gamepad1.left_trigger * 0.7) );
			if( controller1.a.onPress( ) )
				deposit.releaseToggle( );

			if( lift.getMotorPosition() < 70 )
				deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else
				deposit.setAnglePosition( Deposit.AngleStates.DROP_ANGLE );

			telemetry.addData( "lift pos", lift.getMotorPosition( ) );
			telemetry.addData( "release pos", deposit.getReleasePosition( ) );
			telemetry.addData( "angler pos", deposit.getAnglePosition( ) );
			telemetry.update( );
			controller1.update( );
		}
	}
}
