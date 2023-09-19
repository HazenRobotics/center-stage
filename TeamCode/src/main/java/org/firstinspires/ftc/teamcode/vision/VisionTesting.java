package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionTesting extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    PiecePosition piecePosition;

    public enum PiecePosition {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }

    static final Rect leftPos = new Rect(
            new Point(0, 0),
            new Point(426, 720));
    static final Rect midPos = new Rect(
            new Point(426, 0),
            new Point(852, 720));
    static final Rect rightPos = new Rect(
            new Point(852, 0),
            new Point(1278, 720));

    public VisionTesting(Telemetry t) {
        telemetry = t;
    }


    public Mat processFrame(Mat input, String type) {

        Scalar colorLowerBound;
        Scalar colorUpperBound;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (type.equals("Red")) {
            colorLowerBound = new Scalar(0, 40, 40);
            colorUpperBound = new Scalar(20, 255, 255);
        } else {
            colorLowerBound = new Scalar(100, 40, 40);
            colorUpperBound = new Scalar(105, 255, 255);
        }
        final double percentColorThreshold = 0.02;
        Core.inRange(mat, colorLowerBound, colorUpperBound, mat);

        Mat left = mat.submat(leftPos);
        Mat middle = mat.submat(midPos);
        Mat right = mat.submat(rightPos);

        double leftValue = Core.sumElems(left).val[0] / leftPos.area() / 255;


        if (leftValue > percentColorThreshold) {
            piecePosition = PiecePosition.LEFT;
        } else {

            double middleValue = Core.sumElems(middle).val[0] / midPos.area() / 255;
            if (middleValue > percentColorThreshold) {
                piecePosition = PiecePosition.MIDDLE;
            } else {

                double rightValue = Core.sumElems(right).val[0] / rightPos.area() / 255;
                if (rightValue > percentColorThreshold) {

                    piecePosition = PiecePosition.RIGHT;
                } else {
                    piecePosition = PiecePosition.NOT_FOUND;
                }
            }
        }

        left.release();
        middle.release();
        right.release();

        return null;
    }

    public PiecePosition getPiecePosition() {
        return piecePosition;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat redImage = processFrame( input, "Red" );
        Mat blueImage = processFrame( input, "Blue" );
        double elementValue = Core.sumElems( redImage ).val[0] / (redImage.rows( ) * redImage.cols( )) / 255;
        double duckValue = Core.sumElems( blueImage ).val[0] / (blueImage.rows( ) * blueImage.cols( )) / 255;
        telemetry.update( );
        if( elementValue < duckValue )
            return blueImage;
        return redImage;

    }
}
