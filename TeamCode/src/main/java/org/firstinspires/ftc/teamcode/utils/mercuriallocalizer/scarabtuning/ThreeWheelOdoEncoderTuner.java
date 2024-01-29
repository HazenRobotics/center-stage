package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scarabtuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

@Config
@Autonomous(group = "Mercurial")
//@Disabled
public class ThreeWheelOdoEncoderTuner extends LinearOpMode {

	CoaxialSwerveDrive drive;
	Encoder left, right, perp;

	public static double LMult = 1, RMult = 1, PMult = 1;
	@Override
	public void runOpMode( ) throws InterruptedException {
		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
		drive = new CoaxialSwerveDrive( hardwareMap );
		left = new Encoder( hardwareMap.get( DcMotorEx.class, "FLM/paraLEnc" ) ).setDirection( Encoder.Direction.REVERSE );
		right = new Encoder( hardwareMap.get( DcMotorEx.class, "BRM/paraREnc" ) );
		perp = new Encoder( hardwareMap.get( DcMotorEx.class, "BLM/perpEnc" ) );

		left.reset();
		right.reset();
		perp.reset();

		waitForStart();

		while( opModeIsActive() ) {
			drive.drive( 0.03,0,0 );
			telemetry.addData( "left pos", left.getCurrentPosition() * LMult );
			telemetry.addData( "right pos", right.getCurrentPosition()  * RMult );
			telemetry.addData( "perp pos", perp.getCurrentPosition() * PMult);
			telemetry.update();
		}
	}
}
