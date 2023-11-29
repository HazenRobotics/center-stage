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

	double liftPos, liftPower;
	boolean safeToFold = false;
	@Override
	public void runOpMode( ) throws InterruptedException {
		lift = new Lift( hardwareMap );
		deposit = new Deposit( hardwareMap, "release", "angler" );
		controller1 = new GamepadEvents( gamepad1 );

		waitForStart();

		while( opModeIsActive() ) {
			liftPos = lift.getPosition();
			liftPower = gamepad1.right_trigger - (gamepad1.left_trigger * (0.4 + (((620 - liftPos) / 620) * 0.2)));

			lift.setPower( liftPower );
			if( controller1.a.onPress( ) )
				deposit.releaseToggle( );

//			if( deposit.getAnglePosition() == )

			// up to 9, scoringAngle
			// to continue scoring, go down
			// to go back to bucket, go back up to 9, bucketAngle, go down

			if ( liftPower <= -0.3 && liftPos > 180 || liftPos < 100 )
				deposit.setAnglePosition( Deposit.AngleStates.GRAB );
			else
				deposit.setAnglePosition( Deposit.AngleStates.DROP_BACKDROP );

			telemetry.addData( "lift pos", liftPos );
			telemetry.addData( "release pos", deposit.getReleasePosition( ) );
			telemetry.addData( "angler pos", deposit.getAnglePosition( ) );
			telemetry.update( );
			controller1.update( );
		}
	}
}
