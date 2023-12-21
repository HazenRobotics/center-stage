package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BackdropProcessor implements VisionProcessor {

    public enum PixelColor {
        GREEN, PURPLE, YELLOW, WHITE
    }

    public final static double PIXEL_SQUARE_THRESHOLD = 1.5;
    public final static double PIXEL_SIZE_THRESHOLD_LOW = 10;
    public final static double PIXEL_SIZE_THRESHOLD_HIGH = 40;
    public final static double APRIL_SQUARE_THRESHOLD_LOW = 1.1;
    public final static double APRIL_SQUARE_THRESHOLD_HIGH = 2;
    public final static double APRIL_SIZE_THRESHOLD_LOW = 40;
    public final static double APRIL_SIZE_THRESHOLD_HIGH = 100;
    public final static double APRIL_Y_THRESHOLD_HIGH = 450;
    public final static double APRIL_Y_THRESHOLD_LOW = 0;
    public final static int NEAR_SIZE_TORLANCE = 10;

    private final int[] gridRows6 = new int[6];
    private final int[] gridColumns6 = new int[6];
    private final int[] gridRows7 = new int[7];
    private final int[] gridColumns7 = new int[7];


    public Scalar holeLowerBound = new Scalar(0, 0, 0);
    public Scalar holeUpperBound = new Scalar(255, 255, 120);
    Mat pixels = new Mat();

    Mat kernel = Mat.ones(3, 3, CvType.CV_32F);

    ArrayList<Rect> holeRects = new ArrayList<>();
    ArrayList<Rect> aprilRects = new ArrayList<>();
    ArrayList<Rect> noiseRects = new ArrayList<>();
    ArrayList<Rect> allRects = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, pixels, Imgproc.COLOR_RGB2HSV);


        Core.inRange(pixels, holeLowerBound, holeUpperBound, pixels);

        Imgproc.morphologyEx(pixels, pixels, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(pixels, pixels, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 3);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(pixels, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        holeRects.clear();
        aprilRects.clear();
        noiseRects.clear();
        allRects.clear();
        int highestY = 0;
        int lowestY = 1000000;
        for (int i = 0; i < contours.size(); i++) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            allRects.add(boundingRect);
            if (boundingRect.height < boundingRect.width * PIXEL_SQUARE_THRESHOLD && boundingRect.width < boundingRect.height * PIXEL_SQUARE_THRESHOLD && boundingRect.width < PIXEL_SIZE_THRESHOLD_HIGH && boundingRect.width > PIXEL_SIZE_THRESHOLD_LOW && boundingRect.height < PIXEL_SIZE_THRESHOLD_HIGH && boundingRect.height > PIXEL_SIZE_THRESHOLD_LOW) {
                holeRects.add(boundingRect);
                if (boundingRect.y > highestY) {
                    highestY = boundingRect.y;
                }
                if (boundingRect.y < lowestY) {
                    lowestY = boundingRect.y;
                }

            } else if (boundingRect.height > boundingRect.width * APRIL_SQUARE_THRESHOLD_LOW && boundingRect.height < boundingRect.width * APRIL_SQUARE_THRESHOLD_HIGH && boundingRect.width < APRIL_SIZE_THRESHOLD_HIGH && boundingRect.width > APRIL_SIZE_THRESHOLD_LOW && boundingRect.y > APRIL_Y_THRESHOLD_LOW && boundingRect.x < APRIL_Y_THRESHOLD_HIGH) {
                aprilRects.add(boundingRect);
            } else {
                noiseRects.add(boundingRect);
            }
        }
        removeBadPixels(allRects);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        displayRects(holeRects, Color.GREEN, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(noiseRects, Color.RED, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(aprilRects, Color.BLUE, paint, canvas, scaleBmpPxToCanvasPx);


    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    private Rect pointToPointRect(int x1, int x2, int y1, int y2) {
        return new Rect(x1, y1, x2 - x1, y2 - y1);
    }

    public void removeSmallerPixel(Rect r1, Rect r2) {

        Rect removeRect = r1;
        if (aprilRects.contains(r1)) {
            removeRect = r2;
        } else if (aprilRects.contains(r2)) {
        } else if (noiseRects.contains(r1)) {
        } else if (noiseRects.contains(r2)) {
            removeRect = r2;
        } else if (r1.area() > r2.area()) {
            removeRect = r2;
        }
        holeRects.remove(removeRect);


    }

    public boolean isNear(Rect r1, Rect r2) {
        r1 = new Rect(r1.x, r1.y, r1.width + NEAR_SIZE_TORLANCE, r1.height + NEAR_SIZE_TORLANCE);
        r2 = new Rect(r2.x, r2.y, r2.width + NEAR_SIZE_TORLANCE, r2.height + NEAR_SIZE_TORLANCE);

        return (r1.x <= r2.x) && (r2.x < r1.x + r1.width) && (r2.y <= r1.y) && (r1.y < r2.y + r2.width) || (r2.x <= r1.x) && (r1.x < r2.x + r2.width) && (r2.y <= r1.y) && (r1.y < r2.y + r2.width);
    }

    public void createGrid() {
        gridColumns6[0] = holeRects.get(0).y;
        ArrayList<Rect> colounmRects6 = inRow(holeRects.get(0));
        for (int i = 1; i > colounmRects6.size(); i++) {
            gridColumns6[i] = colounmRects6.get(i).x;
            ArrayList<Rect> rowRects6 = inColum(colounmRects6.get(i));
            for (int j = 0; j > rowRects6.size(); j++) {
                gridRows6[j] = rowRects6.get(j).x;
            }
        }
        if (holeRects.size() > colounmRects6.size()) {
            gridColumns7[0] = holeRects.get(colounmRects6.size()).y;
            ArrayList<Rect> colounmRects7 = inRow(holeRects.get(0));
            for (int i = 1; i > colounmRects7.size(); i++) {
                gridColumns7[i] = colounmRects7.get(i).x;
                ArrayList<Rect> rowRects7 = inColum(colounmRects7.get(i));
                for (int j = 0; j > rowRects7.size(); j++) {
                    gridRows7[j] = colounmRects7.get(j).x;
                }
            }
        }

    }

    public boolean outOfGrid(Rect rect) {
        return inColum(rect).isEmpty() && inRow(rect).isEmpty();
    }

    public ArrayList<Rect> inRow(Rect targetRectangle) {
        ArrayList<Rect> rects = new ArrayList<>();
        for (Rect currentRectangle : holeRects) {
            if (((targetRectangle.y - (targetRectangle.height / 2)) <= (currentRectangle.y + NEAR_SIZE_TORLANCE)) ||
                    ((targetRectangle.y - (targetRectangle.height / 2)) >= (currentRectangle.y - NEAR_SIZE_TORLANCE))) {
                rects.add(currentRectangle);
            }
        }
        return rects;
    }

    public ArrayList<Rect> inColum(Rect targetRectangle) {
        ArrayList<Rect> rects = new ArrayList<>();
        for (Rect currentRectangle : holeRects) {
            if (((targetRectangle.x - (targetRectangle.width / 2)) <= (currentRectangle.x + NEAR_SIZE_TORLANCE)) ||
                    ((targetRectangle.x - (targetRectangle.width / 2)) >= (currentRectangle.x - NEAR_SIZE_TORLANCE))) {
                rects.add(currentRectangle);
            }
        }
        return rects;
    }

    public void removeBadPixels(ArrayList<Rect> rects) {
        for (int i = 0; i < rects.size(); i++) {
            for (int j = 0; j < rects.size(); j++) {
                Rect r1 = rects.get(j);
                Rect r2 = rects.get(i);
                if (isNear(r1, r2) && !r2.equals(r1)) {
                    if (holeRects.contains(r1) && outOfGrid(r1)) {
                        holeRects.remove(r1);
                    } else {
                        removeSmallerPixel(r1, r2);
                    }

                }

            }
        }

    }

    public void displayRects(ArrayList<Rect> rects, int color, Paint paint, Canvas canvas, float scaleBmpPxToCanvasPx) {
        paint.setColor(color);
        for (int i = 0; i < rects.size(); i++) {
            if (rects.get(i) != null) {
                canvas.drawRect(makeGraphicsRect(rects.get(i), scaleBmpPxToCanvasPx), paint);
            }
        }

    }

    public ArrayList<Rect> getHoleRects() {
        return holeRects;
    }


}
