package org.firstinspires.ftc.teamcode.vision.pipelines;

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

    public enum PieceColor {
        RED,
        BLUE
    }

    PieceColor pieceColor;

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


    public Mat[] processFrame(Mat input, int a) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar redLowerBound = new Scalar(0, 60, 40);
        Scalar redUpperBound = new Scalar(7, 255, 255);

        Scalar blueLowerBound = new Scalar(100, 60, 40);
        Scalar blueUpperBound = new Scalar(130, 255, 255);

        final double percentColorScalar = 255;
        final double percentColorThreshold = 0.02 * percentColorScalar; //Percent value on left

        Mat matRed = new Mat();
        Mat matBlue = new Mat();

        Core.inRange(mat, redLowerBound, redUpperBound, matRed);
        Core.inRange(mat, blueLowerBound, blueUpperBound, matBlue);

        Mat left = mat.submat(leftPos);
        Mat middle = mat.submat(midPos);
        Mat right = mat.submat(rightPos);

        double leftValue = Core.sumElems(left).val[0] / leftPos.area();


        if (leftValue > percentColorThreshold) {
            piecePosition = PiecePosition.LEFT;
        } else {

            double middleValue = Core.sumElems(middle).val[0] / midPos.area();
            if (middleValue > percentColorThreshold) {
                piecePosition = PiecePosition.MIDDLE;
            } else {

                double rightValue = Core.sumElems(right).val[0] / rightPos.area();
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

        Mat[] newMat = new Mat[2];
        newMat[0] = matRed;
        newMat[1] = matBlue;
        return newMat;
    }

    public PiecePosition getPiecePosition() {
        return piecePosition;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat[] Image = processFrame(input, 1);
        double redValue = Core.sumElems(Image[0]).val[0] / (Image[0].rows() * Image[0].cols());
        double blueValue = Core.sumElems(Image[1]).val[0] / (Image[1].rows() * Image[1].cols());
        telemetry.update();
        if (redValue < blueValue) {
            return Image[1]; //blue
        }
        return Image[0]; //red

    }
}
