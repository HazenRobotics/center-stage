package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scarabtuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

@Autonomous(group = "Mercurial")
//@Disabled
public class ThreeWheelOdoEncoderTuner extends LinearOpMode {

	CoaxialSwerveDrive drive;
	Encoder left, right, perp;
	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new CoaxialSwerveDrive( hardwareMap );
		left = new Encoder( hardwareMap.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.REVERSE );
		right = new Encoder( hardwareMap.get( DcMotorEx.class, "BRM/paraREnc" ) );
		perp = new Encoder( hardwareMap.get( DcMotorEx.class, "BLM/perpEnc" ) );

		left.reset();
		right.reset();
		perp.reset();

		waitForStart();

		while( opModeIsActive() ) {
			drive.drive( 0,0.03,0 );
			telemetry.addData( "left pos", left.getCurrentPosition() );
			telemetry.addData( "right pos", right.getCurrentPosition() );
			telemetry.addData( "perp pos", perp.getCurrentPosition() );
			telemetry.update();
		}
	}
}
