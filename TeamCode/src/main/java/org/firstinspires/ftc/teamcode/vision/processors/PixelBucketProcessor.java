package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class PixelBucketProcessor implements VisionProcessor {
    public enum PixelColor {
        GREEN, PURPLE, YELLOW, WHITE
    }

    Mat green = new Mat( );
    Mat purple = new Mat( );
    Mat yellow = new Mat( );
    Mat white = new Mat( );
    Mat bucket = new Mat();

    Mat kernel = Mat.ones( 3, 3, CvType.CV_32F );

    ArrayList<Rect> greenRects = new ArrayList<>( );
    ArrayList<Rect> purpleRects = new ArrayList<>( );
    ArrayList<Rect> yellowRects = new ArrayList<>( );
    ArrayList<Rect> whiteRects = new ArrayList<>( );
    ArrayList[] pixelsRects = new ArrayList[]{
            greenRects,purpleRects,yellowRects,whiteRects
    };
    public Scalar greenLowerBoundHSV = new Scalar( 40, 85, 40 );
    public Scalar greenUpperBoundHSV = new Scalar( 50, 255, 255 );
    Scalar purpleLowerBound = new Scalar( 122, 23, 102 );
    Scalar purpleUpperBound = new Scalar( 157, 111, 255 );
    Scalar yellowLowerBound = new Scalar( 8, 135, 179 );
    Scalar yellowUpperBound = new Scalar( 45, 255, 255 );
    Scalar whiteLowerBound = new Scalar( 0, 0, 181 );
    Scalar whiteUpperBound = new Scalar( 76, 14, 255 );

    int[] topcords = {0, 0}; //fill this out later
    int[] bottomCords = {0, 0}; //fill this out later
    Mat top = new Mat();
    Mat bottom = new Mat();

    public PixelColor getPixelType(int x, int y) {
        if (white.get(x, y)[0] == 255) {
            return PixelColor.WHITE;
        } else if (green.get(x, y)[0] == 255) {
            return PixelColor.GREEN;
        } else if (purple.get(x, y)[0] == 255) {
            return PixelColor.PURPLE;
        } else if (yellow.get(x, y)[0] == 255) {
            return PixelColor.YELLOW;
        }
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame( Mat frame, long captureTimeNanos ) {
//		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2Lab );
        Imgproc.cvtColor(frame, bucket, Imgproc.COLOR_RGB2HSV);

        Imgproc.morphologyEx(bucket, bucket, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(bucket, bucket, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 4);

        Core.inRange(bucket, greenLowerBoundHSV, greenUpperBoundHSV, green);
        Core.inRange(bucket, purpleLowerBound, purpleUpperBound, purple);
        Core.inRange(bucket, yellowLowerBound, yellowUpperBound, yellow);
//		Core.inRange( temp, whiteLowerBound, whiteUpperBound, white );
    return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        return;
    }

}
