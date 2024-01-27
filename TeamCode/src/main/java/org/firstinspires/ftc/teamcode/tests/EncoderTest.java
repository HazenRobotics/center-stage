package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@TeleOp
public class EncoderTest extends LinearOpMode {

	ArrayList<DcMotorEx> encoders = new ArrayList<>( 5 );

	@Override
	public void runOpMode( ) throws InterruptedException {
		encoders.add( hardwareMap.get( DcMotorEx.class, "FLM/paraLEnc" ) );
		encoders.add( hardwareMap.get( DcMotorEx.class, "BLM/perpEnc" ) );
		encoders.add( hardwareMap.get( DcMotorEx.class, "FRM/liftEnc" ) );
		encoders.add( hardwareMap.get( DcMotorEx.class, "BRM/paraREnc" ) );
		encoders.add( hardwareMap.get( DcMotorEx.class, "climb" ) );

		waitForStart();


		while( opModeIsActive() ) {
			for( int i = 0; i < encoders.size( ); i++ )
				telemetry.addData( "encoder " + i, encoders.get( i ).getCurrentPosition( ) );
			telemetry.update( );
		}
	}
}
