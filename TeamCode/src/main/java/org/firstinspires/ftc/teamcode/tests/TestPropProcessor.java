package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp
public class TestPropProcessor extends LinearOpMode {

	PropProcessor propProcessor;
	VisionPortal visionPortal;

	public static PropProcessor.PropColor color = PropProcessor.PropColor.RED_FAR;

	@Override
	public void runOpMode( ) throws InterruptedException {
		propProcessor = new PropProcessor().setPropColor( PropProcessor.PropColor.RED_FAR );

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get( WebcamName.class, "front"))
				.addProcessor( propProcessor )
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.enableLiveView( true )
				.setAutoStopLiveView(true)
				.build();

		while( opModeInInit() && !opModeIsActive() ) {
			propProcessor.setPropColor( color );

			telemetry.addData( "position", propProcessor.getPropPosition( ) );
			telemetry.addData( "color", propProcessor.getPropColor() );
			telemetry.update( );
		}

		waitForStart();
	}
}
