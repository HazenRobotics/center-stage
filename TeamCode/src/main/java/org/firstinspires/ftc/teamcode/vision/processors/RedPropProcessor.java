package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedPropProcessor implements VisionProcessor {

	public enum PropPosition {
		LEFT, RIGHT, MIDDLE, NOT_FOUND
	}
	PropPosition propPosition;

	public Rect leftPos = new Rect( 35, 100, 145, 120 );
	public Rect midPos = new Rect( 355, 110, 110, 100 );
	private Scalar redLowerBound = new Scalar( 0, 100, 40 );
	private Scalar redUpperBound = new Scalar( 5, 255, 255 );
	private Scalar redLowerBound2 = new Scalar( 175, 100, 40 );
	private Scalar redUpperBound2 = new Scalar( 180, 255, 255 );

	Mat matRed = new Mat( );
	Mat matRed2 = new Mat( );
	Mat temp = new Mat( );
	Mat left;
	Mat middle;

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		Core.inRange( temp, redLowerBound, redUpperBound, matRed );
		Core.inRange( temp, redLowerBound2, redUpperBound2, matRed2 );
		Core.bitwise_or( matRed, matRed2, matRed );
		matRed.copyTo( temp );

		final double percentColorThreshold = 0.1 * 255; //Percent value on left, 255 for max amount of color

		left = temp.submat( leftPos );
		middle = temp.submat( midPos );

		double leftValue = Core.sumElems( left ).val[0] / leftPos.area( );
		double middleValue = Core.sumElems( middle ).val[0] / midPos.area( );

		double maxValue = Math.max( leftValue, middleValue );

		left.release( );
		middle.release( );

		if( maxValue > percentColorThreshold ) {
			if( maxValue == leftValue ) return PropPosition.LEFT;
			if( maxValue == middleValue ) return PropPosition.MIDDLE;
		}

		return PropPosition.RIGHT;
	}

	@Override
	public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
							 float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
		Paint selectedPaint = new Paint( );
		selectedPaint.setColor( Color.GREEN );
		selectedPaint.setStyle( Paint.Style.STROKE );
		selectedPaint.setStrokeWidth( scaleCanvasDensity * 4 );

		Paint nonSelectedPaint = new Paint( );
		nonSelectedPaint.setColor( Color.RED );
		nonSelectedPaint.setStyle( Paint.Style.STROKE );
		nonSelectedPaint.setStrokeWidth( scaleCanvasDensity * 4 );

		android.graphics.Rect drawRectangleLeft = makeGraphicsRect( leftPos, scaleBmpPxToCanvasPx );
		android.graphics.Rect drawRectangleMiddle = makeGraphicsRect( midPos, scaleBmpPxToCanvasPx );

		propPosition = (PropPosition) userContext;
		switch( propPosition ) {
			case LEFT:
				canvas.drawRect( drawRectangleLeft, selectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				break;
			case MIDDLE:
				canvas.drawRect( drawRectangleLeft, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, selectedPaint );
				break;
			case RIGHT:
				canvas.drawRect( drawRectangleLeft, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				break;
		}
	}

	private android.graphics.Rect makeGraphicsRect( Rect rect, float scaleBmpPxToCanvasPx ) {
		int left = Math.round( rect.x * scaleBmpPxToCanvasPx );
		int top = Math.round( rect.y * scaleBmpPxToCanvasPx );
		int right = left + Math.round( rect.width * scaleBmpPxToCanvasPx );
		int bottom = top + Math.round( rect.height * scaleBmpPxToCanvasPx );

		return new android.graphics.Rect( left, top, right, bottom );
	}

	public PropPosition getPiecePosition( ) {
		return propPosition;
	}
}
