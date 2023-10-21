package org.firstinspires.ftc.teamcode.vision.processors;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PixelBucketProcessor extends PixelProcessor implements VisionProcessor {
    int[] topcords = {0, 0}; //fill this out later
    int[] bottomCords = {0, 0}; //fill this out later
    Mat top = new Mat();
    Mat bottom = new Mat();

    public boolean tooManyPixels() {
        int pixelCount = 0;
        for (int i = pixelsRects.length; i > 0; i--) {
            pixelCount += pixelsRects[i].size();
        }
        return pixelCount > 2;
    }

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
    public Object processFrame( Mat frame, long captureTimeNanos ) {
//		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2Lab );
        Imgproc.cvtColor(frame, temp, Imgproc.COLOR_RGB2HSV);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 4);

        Core.inRange(temp, greenLowerBoundHSV, greenUpperBoundHSV, green);
        Core.inRange(temp, purpleLowerBound, purpleUpperBound, purple);
        Core.inRange(temp, yellowLowerBound, yellowUpperBound, yellow);
//		Core.inRange( temp, whiteLowerBound, whiteUpperBound, white );
    return frame;
    }

}
