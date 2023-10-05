package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.util.ElapsedTime;

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
import java.util.Arrays;
import java.util.List;

public class PixelProcessor implements VisionProcessor {

	public enum PixelColor {
		GREEN, PURPLE, YELLOW, WHITE
	}

	PixelColor color = PixelColor.GREEN;
	Scalar greenLowerBound = new Scalar( 42.5, 85, 40 );
	Scalar greenUpperBound = new Scalar( 55, 255, 255 );
	Scalar purpleLowerBound = new Scalar( 122, 33, 142 );
	Scalar purpleUpperBound = new Scalar( 136, 125, 255 );
	Scalar yellowLowerBound = new Scalar( 20, 143, 179 );
	Scalar yellowUpperBound = new Scalar( 28, 255, 255 );
	public Scalar whiteLowerBound = new Scalar( 0, 0, 240 );
	public Scalar whiteUpperBound = new Scalar( 1, 5, 255 );
	public Scalar lowerBound;
	public Scalar upperBound;

	Mat temp = new Mat( );

	Mat kernel = Mat.ones(3,3, CvType.CV_32F);

	double minRow = 0, maxRow = 0, minCol = 0, maxCol = 0;

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		switch( color ) {
			case GREEN:
				lowerBound = greenLowerBound;
				upperBound = greenUpperBound;
				break;
			case PURPLE:
				lowerBound = purpleLowerBound;
				upperBound = purpleUpperBound;
				break;
			case YELLOW:
				lowerBound = yellowLowerBound;
				upperBound = yellowUpperBound;
				break;
			case WHITE:
				lowerBound = whiteLowerBound;
				upperBound = whiteUpperBound;
				break;
		}

		Core.inRange( temp, lowerBound, upperBound, frame );

		Imgproc.morphologyEx( frame, frame, Imgproc.MORPH_ERODE, kernel, new Point(0,0), 3  );
		Imgproc.morphologyEx( frame, frame, Imgproc.MORPH_DILATE, kernel, new Point(0,0), 3  );

//		minRow = frame.rows();
//		minCol = frame.cols();
//
//		for( int i = 0; i < frame.rows(); i++ ) {
//			for( int j = 0; j < frame.cols(); j++ ) {
//				if (frame.get(i, j)[0] != 0) {
//					if (i < minRow)
//						minRow = i;
//					if (j < minCol)
//						minRow = j;
//					if (i > maxRow)
//						maxRow = i;
//					if (j > maxCol)
//						maxRow = j;
//				}
//			}
//		}


		return frame;
	}

	@Override
	public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
							 float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
		Paint selectedPaint = new Paint( );
		selectedPaint.setColor( Color.GREEN );
		selectedPaint.setStyle( Paint.Style.STROKE );
		selectedPaint.setStrokeWidth( scaleCanvasDensity * 4 );

		android.graphics.Rect rect = makeGraphicsRect( new Rect( new Point(minRow, minCol), new Point(maxRow, maxCol) ), scaleBmpPxToCanvasPx );
		canvas.drawRect( rect, selectedPaint );

	}

	private android.graphics.Rect makeGraphicsRect( Rect rect, float scaleBmpPxToCanvasPx ) {
		int left = Math.round( rect.x * scaleBmpPxToCanvasPx );
		int top = Math.round( rect.y * scaleBmpPxToCanvasPx );
		int right = left + Math.round( rect.width * scaleBmpPxToCanvasPx );
		int bottom = top + Math.round( rect.height * scaleBmpPxToCanvasPx );

		return new android.graphics.Rect( left, top, right, bottom );
	}

}
