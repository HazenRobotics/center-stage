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

	public Rect midPos = new Rect( 240, 175, 100, 100 );
	public Rect rightPos = new Rect( 530, 155, 105, 125 );
	private Scalar blueLowerBound = new Scalar( 100, 60, 40 );
	private Scalar blueUpperBound = new Scalar( 130, 255, 255 );
	Mat matBlue = new Mat( );
	Mat temp = new Mat();
	Mat middle;
	Mat right;

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		Core.inRange( temp, blueLowerBound, blueUpperBound, matBlue );
		matBlue.copyTo(temp);

		final double percentColorThreshold = 0.1 * 255; //Percent value on left, 255 for max amount of color

		right = temp.submat( rightPos );
		middle = temp.submat( midPos );

		double rightValue = Core.sumElems( right ).val[0] / rightPos.area( );
		double middleValue = Core.sumElems( middle ).val[0] / midPos.area( );

		double maxValue = Math.max( rightValue, middleValue );

		right.release( );
		middle.release( );

		if (maxValue > percentColorThreshold) {
			if( maxValue == rightValue ) return PropPosition.RIGHT;
			if( maxValue == middleValue ) return PropPosition.MIDDLE;
		}

		return PropPosition.LEFT;
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

		android.graphics.Rect drawRectangleRight = makeGraphicsRect( rightPos, scaleBmpPxToCanvasPx );
		android.graphics.Rect drawRectangleMiddle = makeGraphicsRect( midPos, scaleBmpPxToCanvasPx );

		propPosition = (PropPosition) userContext;
		switch( propPosition ) {
			case LEFT:
				canvas.drawRect( drawRectangleRight, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, nonSelectedPaint );
				break;
			case MIDDLE:
				canvas.drawRect( drawRectangleRight, nonSelectedPaint );
				canvas.drawRect( drawRectangleMiddle, selectedPaint );
				break;
			case RIGHT:
				canvas.drawRect( drawRectangleRight, selectedPaint );
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
