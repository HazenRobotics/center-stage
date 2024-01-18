/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point3;

import java.util.ArrayList;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Config
//@Disabled
public class AprilTagUtil {
    private AprilTagProcessor frontATP;
    //    private AprilTagProcessor backATP;
//    public VisionPortal backVP;
    public VisionPortal frontVP;
    private final double TAG_ANGLE_OFFSET = 1.59807621135;

    public static double FRONT_X_OFFSET = -7;
    public static double FRONT_Y_OFFSET = /*-2.5*/ 0;
    public static double BACK_X_OFFSET = /*7*/ 0;
    public static double BACK_Y_OFFSET = /*-5.5*/ 0;

    final static Point3[] APRIL_TAG_BOARD_POSTIONS = {
            new Point3(60.25f, 41.41f, 4f),
            new Point3(60.25f, 35.41f, 4f),
            new Point3(60.25f, 29.41f, 4f),
            new Point3(60.25f, -29.41f, 4f),
            new Point3(60.25f, -35.41f, 4f),
            new Point3(60.25f, -41.41f, 4f),
    };
    //Bigs are the 5.5s and 4s are the small
    final static Point3[] APRIL_TAG_WALL_POSTIONS = {
            new Point3(-70.25f, -40.625f, 5.5f),
            new Point3(-70.25f, -35.125f, 4f),
            new Point3(-70.25f, 35.125f, 4f),
            new Point3(-70.25f, 40.625f, 5.5f),
    };


    /**
     * The variable to store our instance of the vision portal.
     **/

    /**
     * Initialize the AprilTag processor.
     */
    public static Point3 getTagPosition(int i) {
        if (i > APRIL_TAG_BOARD_POSTIONS.length) {
            i = i - APRIL_TAG_BOARD_POSTIONS.length;
            return APRIL_TAG_WALL_POSTIONS[i - 1];
        } else return APRIL_TAG_BOARD_POSTIONS[i - 1];
    }


    public AprilTagUtil(HardwareMap hw) {


        // Create the AprilTag processor.
        frontATP = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(627.866405467, 627.866405467, 367.632040506, 237.041562849)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

//        backATP = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//
//                // == CAMERA CALIBRATION ==
//                // If you do not manually specify calibration parameters, the SDK will attempt
//                // to load a predefined calibration for your camera.
//                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//
//                // ... these parameters are fx, fy, cx, cy.
//
//                .build();

        // Create the vision portal by using a builder

        // Set the camera (webcam vs. built-in RC phone camera).


        // Choose a camera resolution. Not all cameras support all resolutions.

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.

//        backVP = new VisionPortal.Builder()
//                .setCamera(hw.get(WebcamName.class, "back"))
//                .addProcessor(backATP)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setAutoStopLiveView(true)
//                .enableLiveView( false )
//                .build();

        frontVP = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "front"))
                .addProcessor(frontATP)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

        // Disable or re-enable the aprilTag processor at any time.
//        backVP.setProcessorEnabled(backATP, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {

//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public Point3 getPositionBasedOnTag() {
        ArrayList<AprilTagDetection> detections = new ArrayList<>();

        double xOffset = 0;
        double yOffset = 0;

//        if (backVP.getProcessorEnabled(backATP)) {
//            detections.addAll(backATP.getDetections());
//            xOffset = BACK_X_OFFSET;
//            yOffset = BACK_Y_OFFSET;
//        }
        if (frontVP.getProcessorEnabled(frontATP)) {
            detections.addAll(frontATP.getDetections());
            xOffset = FRONT_X_OFFSET;
            yOffset = FRONT_Y_OFFSET;
        }

        if (detections.isEmpty()) return new Point3(0, 0, 0);
        else {
            double x = 0;
            double y = 0;
            double yaw = 0;


            for (int i = 0; i < detections.size(); i++) {
                Point3 curPose = getPositionBasedOnTag(detections.get(i));
                x += curPose.x;
                y += curPose.y;
                yaw += curPose.z;
            }

            x /= detections.size();
            y /= detections.size();
            yaw /= detections.size();

            return new Point3(x + xOffset, y + yOffset, yaw);
        }

    }

    public Point3 getPositionBasedOnTag(AprilTagDetection detection) {

        AprilTagPoseFtc robotPose = detection.ftcPose;
        Point3 tagpos = getTagPosition(detection.id);

        double x = tagpos.x - (tagpos.x > 0 ? robotPose.x : -robotPose.x);
        double y = tagpos.y - (tagpos.y > 0 ? robotPose.y : -robotPose.y);
        double yaw = robotPose.yaw;

        if (detection.id < 7) {
            x += TAG_ANGLE_OFFSET;
            yaw = robotPose.yaw + Math.PI;
        }
        return new Point3(x, y, yaw);

    }

    public void setLensIntrinsics(boolean isFront, double fx, double fy, double cx, double cy) {
        if (isFront) {
            frontATP = new AprilTagProcessor.Builder().setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(fx, fy, cx, cy)
                    .build();
        }
//        else {
//             backATP=new AprilTagProcessor.Builder().setDrawAxes(true)
//                    .setDrawCubeProjection(true)
//                    .setDrawTagOutline(true)
//                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                    .setLensIntrinsics(fx,fy,cx,cy)
//                    .build();
//        }

    }

    public void setLensIntrinsics(boolean isFront, double[] config) {
        if (isFront) {
            frontATP = new AprilTagProcessor.Builder().setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(config[0], config[1], config[2], config[3])
                    .build();
        }
//        else {
//             backATP=new AprilTagProcessor.Builder().setDrawAxes(true)
//                    .setDrawCubeProjection(true)
//                    .setDrawTagOutline(true)
//                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                    .setLensIntrinsics(fx,fy,cx,cy)
//                    .build();
//        }

    }
}   // end class
