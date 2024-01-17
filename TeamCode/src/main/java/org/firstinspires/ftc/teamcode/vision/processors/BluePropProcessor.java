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

public class BluePropProcessor implements VisionProcessor {

	public enum PropPosition {
		LEFT, RIGHT, MIDDLE, NOT_FOUND
	}
	PropPosition propPosition;

	public Rect leftPos = new Rect( 290, 320, 250, 230 );
	public Rect midPos = new Rect( 930, 340, 200, 200 );
	public Rect rightPos = new Rect( 1500, 320, 300, 250 );
	private final Scalar blueLowerBound = new Scalar( 100, 60, 40 );
	private final Scalar blueUpperBound = new Scalar( 130, 255, 255 );
	Mat matBlue = new Mat( );
	Mat temp = new Mat();
	Mat left, middle, right;

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		Core.inRange( temp, blueLowerBound, blueUpperBound, temp );
//		frame.copyTo(temp);

		final double percentColorThreshold = 0.1 * 255; //Percent value on left, 255 for max amount of color

		left = temp.submat( leftPos );
		middle = temp.submat( midPos );
		right = temp.submat( rightPos );

		double leftValue = Core.sumElems( left ).val[0] / leftPos.area( );
		double middleValue = Core.sumElems( middle ).val[0] / midPos.area( );
		double rightValue = Core.sumElems( right ).val[0] / rightPos.area( );

		double maxValue = Math.max( rightValue, Math.max( middleValue, leftValue ) );

		left.release();
		middle.release( );
		right.release();

		if (maxValue > percentColorThreshold) {
			if( maxValue == leftValue ) return PropPosition.LEFT;
			if( maxValue == middleValue ) return PropPosition.MIDDLE;
			if( maxValue == rightValue ) return PropPosition.RIGHT;
		}

		return PropPosition.NOT_FOUND;
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
		android.graphics.Rect drawRectangleRight = makeGraphicsRect( rightPos, scaleBmpPxToCanvasPx );

		propPosition = (PropPosition) userContext;
		switch( propPosition ) {
			case LEFT:
				canvas.drawRect( drawRectangleLeft, selectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				canvas.drawRect( drawRectangleRight, nonSelectedPaint );
				break;
			case MIDDLE:
				canvas.drawRect( drawRectangleLeft, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, selectedPaint );
				canvas.drawRect( drawRectangleRight, nonSelectedPaint );
				break;
			case RIGHT:
				canvas.drawRect( drawRectangleLeft, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				canvas.drawRect( drawRectangleRight, selectedPaint );
				break;
			case NOT_FOUND:
				canvas.drawRect( drawRectangleLeft, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				canvas.drawRect( drawRectangleRight, nonSelectedPaint );
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
