package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class TestVisionOpMode extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private RedPropProcessor redPropProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        redPropProcessor = new RedPropProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor );

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        while (opModeIsActive()) {
            RedPropProcessor.PropPosition position = redPropProcessor.getPiecePosition( );
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}