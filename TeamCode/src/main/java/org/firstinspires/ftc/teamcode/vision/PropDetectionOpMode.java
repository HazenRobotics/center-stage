package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PropDetectionOpMode extends LinearOpMode {
	VisionPortal visionPortal;
	RedPropProcessor redPropProcessor;
	@Override
	public void runOpMode( ) throws InterruptedException {
		redPropProcessor = new RedPropProcessor();

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.addProcessor( redPropProcessor )
				.setCameraResolution(new Size(640, 360))
				.setStreamFormat(VisionPortal.StreamFormat.YUY2)
				.setAutoStopLiveView(true)
				.build();

		while( opModeInInit() && !opModeIsActive() ) {
			telemetry.addData( "position", redPropProcessor.getPiecePosition() );
			telemetry.update();
		}
	}
}
