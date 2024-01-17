package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.PixelProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@Autonomous
@Disabled
public class HomographyOpMode extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private PixelProcessor pixelProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        pixelProcessor = new PixelProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), pixelProcessor);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        while (opModeIsActive()) {
            double[] position = pixelProcessor.getPixelPos( );
            if (position.length > 0) {
                double[] calculatedPos = HomographyTargetDistance.positionFromPoint( new Point(position[1], position[0]) );
                telemetry.addData( "position y", calculatedPos[0] );
                telemetry.addData( "position x", calculatedPos[1] );
            }
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}