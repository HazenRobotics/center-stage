package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PropDetectionOpMode extends LinearOpMode {
	VisionPortal visionPortal;
	PropProcessor propProcessor;
	@Override
	public void runOpMode( ) throws InterruptedException {
		propProcessor = new PropProcessor();

		propProcessor.setPropColor( PropProcessor.PropColor.BLUE );

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.addProcessor(propProcessor)
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.YUY2)
				.setAutoStopLiveView(true)
				.build();

		while( opModeInInit() && !opModeIsActive() ) {
			telemetry.addData( "position", propProcessor.getPiecePosition() );
		}
	}
}
