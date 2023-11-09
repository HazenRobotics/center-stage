package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.PixelProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@Autonomous
public class TestVisionOpMode extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private PropProcessor propProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        propProcessor = new PropProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), propProcessor);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        while (opModeIsActive()) {
            PropProcessor.PropPosition position = propProcessor.getPiecePosition( );
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}