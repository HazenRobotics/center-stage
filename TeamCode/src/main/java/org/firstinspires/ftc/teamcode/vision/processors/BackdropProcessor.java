package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.DeciderPixel;
import org.firstinspires.ftc.teamcode.vision.Pixel;
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
import java.util.Comparator;
import java.util.List;

public class BackdropProcessor implements VisionProcessor {


    public static int PIXEL_SIZE_THRESHOLD_LOW = 5;
    public static int PIXEL_SIZE_THRESHOLD_HIGH = 40;
    public final Scalar greenLowerBound = new Scalar(41, 29.8, 73);
    public final Scalar greenUpperBound = new Scalar(55, 114, 141);
    public final Scalar purpleLowerBound = new Scalar(120, 41, 121);
    public final Scalar purpleUpperBound = new Scalar(125, 67, 161);
    public final Scalar yellowLowerBound = new Scalar(0, 109, 141);
    public final Scalar yellowUpperBound = new Scalar(42, 148, 208);

    public final Scalar holeLowerBound = new Scalar(0, 0, 0);
    public final Scalar holeUpperBound = new Scalar(255, 213, 139);
    public final Scalar zoneLowerBound = new Scalar(0, 0, 0);
    public final Scalar zoneUpperBound = new Scalar(255, 255, 153);

    Mat temp = new Mat();
    Mat green = new Mat();
    Mat purple = new Mat();
    Mat yellow = new Mat();
    Mat zone = new Mat();

    Mat kernel = Mat.ones(3, 3, CvType.CV_32F);

    ArrayList<Rect> greenRects = new ArrayList<>();
    ArrayList<Rect> purpleRects = new ArrayList<>();
    ArrayList<Rect> yellowRects = new ArrayList<>();


    Mat pixels = new Mat();

    ArrayList<Pixel> pixelArrayList = new ArrayList<>();
    Rect zoneRect = new Rect();

    ArrayList<Rect> noiseRects = new ArrayList<>();
    ArrayList<Rect> badRects = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, temp, Imgproc.COLOR_RGB2HSV);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 4);

        Core.inRange(temp, holeLowerBound, holeUpperBound, pixels);
        Core.inRange(temp, greenLowerBound, greenUpperBound, green);
        Core.inRange(temp, purpleLowerBound, purpleUpperBound, purple);
        Core.inRange(temp, yellowLowerBound, yellowUpperBound, yellow);
        Core.inRange(temp, zoneLowerBound, zoneUpperBound, zone);

        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> zones = new ArrayList<>();
        Mat hierarchy = new Mat();

        findBoundingBoxes(green, greenRects, contours);
        findBoundingBoxes(purple, purpleRects, contours);
        findBoundingBoxes(yellow, yellowRects, contours);
        Imgproc.findContours(pixels, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(zone, zones, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        pixelArrayList.clear();
        noiseRects.clear();
        zoneRect = new Rect();

        for (int i = 0; i < zones.size(); i++) {
            Rect boundingRect = Imgproc.boundingRect(zones.get(i));
            if (boundingRect.area() > zoneRect.area() && boundingRect.height < 500 && boundingRect.width < 600) {
                zoneRect = boundingRect;
            }
        }

        for (int i = 0; i < contours.size(); i++) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            if (zoneRect.contains(boundingRect.tl()) && zoneRect.contains(boundingRect.br()) && !zoneRect.equals(boundingRect) && (boundingRect.height > PIXEL_SIZE_THRESHOLD_LOW && boundingRect.height < PIXEL_SIZE_THRESHOLD_HIGH)) {
                pixelArrayList.add(new Pixel(boundingRect, greenRects, purpleRects, yellowRects));
            } else if (!zoneRect.equals(boundingRect)) {
                noiseRects.add(boundingRect);
            }
        }
        removeBadPixels(pixelArrayList);

        ArrayList<Pixel[]> grid = grid(pixelArrayList);
        for (int i = 0; i < grid.size(); i++) {
            for (int j = 0; j < grid.get(i).length; j++) {
                if (!pixelArrayList.contains(grid.get(i)[j])) {
                    pixelArrayList.add(grid.get(i)[j]);
                }
            }
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(zoneRect, scaleBmpPxToCanvasPx), paint);
        displayPixels(pixelArrayList, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(noiseRects, Color.RED, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(badRects, Color.RED, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(greenRects, Color.CYAN, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(purpleRects, Color.CYAN, paint, canvas, scaleBmpPxToCanvasPx);
        displayRects(yellowRects, Color.CYAN, paint, canvas, scaleBmpPxToCanvasPx);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }


    public void removeSmallerPixel(Pixel p1, Pixel p2) {
        if (p1.getRect().area() > p2.getRect().area()) {
            pixelArrayList.remove(p2);
            badRects.add(p2.getRect());
        } else {
            pixelArrayList.remove(p1);
            badRects.add(p1.getRect());
        }

    }


    public void removeBadPixels(ArrayList<Pixel> pixels) {
        badRects.clear();
        for (int i = 0; i < pixels.size(); i++) {
            pixels.size();
            if (pixels.get(i).isTouchingBorder(zoneRect)) {
                badRects.add(pixels.get(i).getRect());
                pixels.remove(pixels.get(i));
            }

        }
        for (int i = 0; i < pixels.size(); i++) {
            for (int j = 0; j < pixels.size(); j++) {
                while (i >= pixels.size()) i--;
                Pixel r1 = pixels.get(j);
                Pixel r2 = pixels.get(i);
                if (r1.isNear(r2.getRect()) && !r2.equals(r1)) {
                    removeSmallerPixel(r1, r2);
                }

            }

        }

    }


    public void displayRects(ArrayList<Rect> rects, int color, Paint paint, Canvas canvas,
                             float scaleBmpPxToCanvasPx) {
        paint.setColor(color);
        if (!rects.isEmpty()) {
            for (int i = 0; i < rects.size(); i++) {
                if (rects.get(i) != null) {

                    canvas.drawRect(makeGraphicsRect(rects.get(i), scaleBmpPxToCanvasPx), paint);
                }
            }
        }
    }

    public void displayPixels(ArrayList<Pixel> pixels, Paint paint, Canvas canvas, float scaleBmpPxToCanvasPx) {
        if (!pixels.isEmpty()) {
            for (int i = 0; i < pixels.size(); i++) {
                Pixel pixel = pixels.get(i);
                if (pixel != null) {
                    paint.setColor(pixel.getGraphicColor());
                    canvas.drawRect(makeGraphicsRect(pixel.getRect(), scaleBmpPxToCanvasPx), paint);
                }
            }
        }
    }

    public ArrayList<Pixel[]> getGrid() {
        return grid(pixelArrayList);
    }


    public ArrayList<Pixel[]> grid(ArrayList<Pixel> pixels) {
        ArrayList<ArrayList<Pixel>> arr1 = new ArrayList<>();
        int maxItems = 7;
        while (!pixels.isEmpty()) {
            ArrayList<Pixel> neighbors = pixels.get(0).rowNeighbors(pixels);
            neighbors.add(pixels.get(0));


            pixels.sort(Comparator.comparingDouble(p -> -p.getRect().x));
            maxItems = (maxItems == 6) ? 7 : 6;

            if (neighbors.size() > maxItems) {
                neighbors.subList(maxItems, neighbors.size()).clear(); // Ensure neighbors does not exceed maxItems
            }

            arr1.add(neighbors);
            pixels.removeAll(neighbors);
        }
        return  positionGird(arr1);
    }

    public ArrayList<Pixel[]> positionGird(ArrayList<ArrayList<Pixel>> grid) {
        ArrayList<Pixel[]> pixels =new ArrayList<>();
        int maxItems = 7;
        for (int i = 0; i < grid.size(); i++) {
            maxItems = (maxItems == 6) ? 7 : 6;
            for (int j = 0; j < grid.get(i).size() && !(grid.get(i).size()== maxItems); j++) {
                int realPostion = 0;
                if (maxItems == 6) {
                    int segmentsize = (zoneRect.width - 40) / 6;
                    while (grid.get(i).get(j).getRect().x < zoneRect.x + (segmentsize * realPostion) + 20)
                        realPostion++;
                }
                if (maxItems == 7) {
                    int segmentsize = zoneRect.width / 7;
                    while (grid.get(i).get(j).getRect().x < zoneRect.x + (segmentsize * realPostion))
                        realPostion++;
                }
                while (realPostion>j) {
                    grid.get(i).add(new Pixel(new Rect(0,0,10,10), DeciderPixel.Color.NO_PIXEL));
                    i++;
                }
            }
        }
        for(int i=0; i<grid.size(); i++) {
            pixels.add((Pixel[]) grid.stream().toArray());
        }
        return  pixels;
    }


    private void findBoundingBoxes(Mat mat, ArrayList<Rect> rects, List<MatOfPoint> contourList) {
        Imgproc.findContours(mat, contourList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        rects.clear();
        for (int i = 0; i < contourList.size(); i++) {
            MatOfPoint point = contourList.get(i);
            Rect boundingRect = Imgproc.boundingRect(point);
            if (boundingRect.area() > 500) {
                rects.add(changeScale(boundingRect, 0.5));
            }
        }
        contourList.clear();
    }

    public static Rect changeScale(Rect rect, double factor) {
        // Calculate the new width and height
        int newWidth = (int) (rect.width * factor);
        int newHeight = (int) (rect.height * factor);
        // The top-left point changes
        int newX = rect.x + (rect.width - newWidth);
        int newY = rect.y + (rect.height - newHeight);
        // Create and return the new Rect
        return new Rect(newX, newY, newWidth, newHeight);
    }
}
