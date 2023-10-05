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

public class PixelProcessor implements VisionProcessor {

	public enum PixelColor {
		GREEN, PURPLE, YELLOW, WHITE
	}

	PixelColor color = PixelColor.GREEN;
	public Scalar greenLowerBound = new Scalar( 40, 85, 40 );
	public Scalar greenUpperBound = new Scalar( 50, 255, 255 );
	public Scalar purpleLowerBound = new Scalar( 122, 23, 102 );
	public Scalar purpleUpperBound = new Scalar( 157, 111, 255 );
	public Scalar yellowLowerBound = new Scalar( 8.5, 135, 179 );
	public Scalar yellowUpperBound = new Scalar( 45, 255, 255 );
	public Scalar whiteLowerBound = new Scalar( 0, 0, 181 );
	public Scalar whiteUpperBound = new Scalar( 76, 14, 255 );

	Mat temp = new Mat( );
	Mat green = new Mat( );
	Mat purple = new Mat( );
	Mat yellow = new Mat( );
	Mat white = new Mat( );

	Mat kernel = Mat.ones( 3, 3, CvType.CV_32F );

	ArrayList<Rect> greenRects = new ArrayList<>( );
	ArrayList<Rect> purpleRects = new ArrayList<>( );
	ArrayList<Rect> yellowRects = new ArrayList<>( );
	ArrayList<Rect> whiteRects = new ArrayList<>( );

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

//		switch( color ) {
//			case GREEN:
//				lowerBound = greenLowerBound;
//				upperBound = greenUpperBound;
//				break;
//			case PURPLE:
//				lowerBound = purpleLowerBound;
//				upperBound = purpleUpperBound;
//				break;
//			case YELLOW:
//				lowerBound = yellowLowerBound;
//				upperBound = yellowUpperBound;
//				break;
//			case WHITE:
//				lowerBound = whiteLowerBound;
//				upperBound = whiteUpperBound;
//				break;
//		}

		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_ERODE, kernel, new Point( 0, 0 ), 3 );
		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_DILATE, kernel, new Point( 0, 0 ), 4 );

		Core.inRange( temp, greenLowerBound, greenUpperBound, green );
		Core.inRange( temp, purpleLowerBound, purpleUpperBound, purple );
		Core.inRange( temp, yellowLowerBound, yellowUpperBound, yellow );
		Core.inRange( temp, whiteLowerBound, whiteUpperBound, white );

		List<MatOfPoint> contours = new ArrayList<>( );
		Mat hierarchy = new Mat( );
		Imgproc.findContours( green, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

		greenRects.clear();
		for( int i = 0; i < contours.size( ); i++ ) {
			MatOfPoint point = contours.get( i );
			Rect boundingRect = Imgproc.boundingRect( point );
			greenRects.add( boundingRect );
		}

		contours.clear();

		Imgproc.findContours( purple, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

		purpleRects.clear();
		for( int i = 0; i < contours.size( ); i++ ) {
			MatOfPoint point = contours.get( i );
			Rect boundingRect = Imgproc.boundingRect( point );
			purpleRects.add( boundingRect );
		}

		contours.clear();

		Imgproc.findContours( yellow, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

		yellowRects.clear();
		for( int i = 0; i < contours.size( ); i++ ) {
			MatOfPoint point = contours.get( i );
			Rect boundingRect = Imgproc.boundingRect( point );
			yellowRects.add( boundingRect );
		}

		contours.clear();

		Imgproc.findContours( white, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

		whiteRects.clear();
		for( int i = 0; i < contours.size( ); i++ ) {
			MatOfPoint point = contours.get( i );
			Rect boundingRect = Imgproc.boundingRect( point );
			whiteRects.add( boundingRect );
		}

		return frame;
	}

	@Override
	public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
							 float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
		Paint paint = new Paint( );
		paint.setColor( Color.GREEN );
		paint.setStyle( Paint.Style.STROKE );
		paint.setStrokeWidth( scaleCanvasDensity * 4 );

		int minSize = 2500;

		for( int i = 0; i < greenRects.size( ); i++ ) {
			if (greenRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect( greenRects.get( i ), scaleBmpPxToCanvasPx );
				if (rect.width() * rect.height() > minSize)
					canvas.drawRect( rect, paint );
			}
		}

		paint.setColor( Color.MAGENTA );

		for( int i = 0; i < purpleRects.size( ); i++ ) {
			if (purpleRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect( purpleRects.get( i ), scaleBmpPxToCanvasPx );
				if (rect.width() * rect.height() > minSize)
					canvas.drawRect( rect, paint );
			}
		}

		paint.setColor( Color.YELLOW );

		for( int i = 0; i < yellowRects.size( ); i++ ) {
			if (yellowRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect( yellowRects.get( i ), scaleBmpPxToCanvasPx );
				if (rect.width() * rect.height() > minSize)
					canvas.drawRect( rect, paint );
			}
		}

		paint.setColor( Color.WHITE );

		for( int i = 0; i < whiteRects.size( ); i++ ) {
			if (whiteRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect( whiteRects.get( i ), scaleBmpPxToCanvasPx );
				if (rect.width() * rect.height() > minSize )
					canvas.drawRect( rect, paint );
			}
		}

	}

	private android.graphics.Rect makeGraphicsRect( Rect rect, float scaleBmpPxToCanvasPx ) {
		int left = Math.round( rect.x * scaleBmpPxToCanvasPx );
		int top = Math.round( rect.y * scaleBmpPxToCanvasPx );
		int right = left + Math.round( rect.width * scaleBmpPxToCanvasPx );
		int bottom = top + Math.round( rect.height * scaleBmpPxToCanvasPx );

		return new android.graphics.Rect( left, top, right, bottom );
	}


}
