package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scarabtuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

@Autonomous
@Disabled
public class ThreeWheelOdoEncoderTuner extends LinearOpMode {

	Encoder left, right, perp;
	@Override
	public void runOpMode( ) throws InterruptedException {
		left = new Encoder( hardwareMap.get( DcMotorEx.class, "FLM/paraLEnc" ) );
		right = new Encoder( hardwareMap.get( DcMotorEx.class, "BRM/paraREnc" ) );
		perp = new Encoder( hardwareMap.get( DcMotorEx.class, "BLM/perpEnc" ) );

		left.reset();
		right.reset();
		perp.reset();

		waitForStart();

		while( opModeIsActive() ) {
			telemetry.addData( "left pos", left.getCurrentPosition() );
			telemetry.addData( "right pos", right.getCurrentPosition() );
			telemetry.addData( "perp pos", perp.getCurrentPosition() );
			telemetry.update();
		}
	}
}
