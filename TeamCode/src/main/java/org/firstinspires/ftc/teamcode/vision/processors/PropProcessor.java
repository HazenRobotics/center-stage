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

public class PropProcessor implements VisionProcessor {

	public enum PropPosition {
		LEFT, RIGHT, MIDDLE, NOT_FOUND
	}
	public enum PropColor {
		RED, BLUE
	}
	PropColor propColor = PropColor.BLUE;
	PropPosition propPosition;

	private final Rect leftPos = new Rect( 0, 0, 420, 720 );
	private final Rect midPos = new Rect( 424, 0, 420, 720 );
	private final Rect rightPos = new Rect( 848, 0, 420, 720 );

	final Scalar redLowerBound = new Scalar( 0, 100, 40 );
	final Scalar redUpperBound = new Scalar( 5, 255, 255 );
	final Scalar redLowerBound2 = new Scalar( 175, 100, 40 );
	final Scalar redUpperBound2 = new Scalar( 180, 255, 255 );
	final Scalar blueLowerBound = new Scalar( 100, 60, 40 );
	final Scalar blueUpperBound = new Scalar( 130, 255, 255 );

	Mat matRed = new Mat( );
	Mat matRed2 = new Mat( );
	Mat matBlue = new Mat( );

	Mat left;
	Mat middle;
	Mat right;

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, frame, Imgproc.COLOR_RGB2HSV );

		if (propColor == PropColor.BLUE) {
			Core.inRange( frame, blueLowerBound, blueUpperBound, matBlue );
			matBlue.copyTo(frame);
		} else if( propColor == PropColor.RED ) {
			Core.inRange( frame, redLowerBound, redUpperBound, matRed );
			Core.inRange( frame, redLowerBound2, redUpperBound2, matRed2 );
			Core.bitwise_or( matRed, matRed2, matRed );
			matRed.copyTo(frame);
		}

		final double percentColorThreshold = 0.02 * 255; //Percent value on left, 255 for max amount of color

		left = frame.submat( leftPos );
		middle = frame.submat( midPos );
		right = frame.submat( rightPos );

		double leftValue = Core.sumElems( left ).val[0] / leftPos.area( );
		if( leftValue > percentColorThreshold ) {
			left.release( );
			middle.release( );
			right.release( );
			return PropPosition.LEFT;
		}

		double middleValue = Core.sumElems( middle ).val[0] / midPos.area( );
		if( middleValue > percentColorThreshold ) {
			left.release( );
			middle.release( );
			right.release( );
			return PropPosition.MIDDLE;
		}

		double rightValue = Core.sumElems( right ).val[0] / midPos.area( );
		if( rightValue > percentColorThreshold ) {
			left.release( );
			middle.release( );
			right.release( );
			return PropPosition.RIGHT;
		}

		left.release( );
		middle.release( );
		right.release( );
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
