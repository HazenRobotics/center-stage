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

	PixelColor color = PixelColor.GREEN;
	public Scalar holeLowerBound = new Scalar( 0, 116, 119 );
	public Scalar holeUpperBound = new Scalar( 146, 255, 136 );
	public Rect rect = new Rect(140, 0, 920, 730);

	Mat temp = new Mat();

	Mat kernel = Mat.ones( 3, 3, CvType.CV_32F );

	ArrayList<Rect> holeRects = new ArrayList<>( );

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
//		Imgproc.cvtColor( frame.submat( rect ), temp, Imgproc.COLOR_RGB2Lab );
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2Lab );

		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_ERODE, kernel, new Point( 0, 0 ), 3 );
		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_DILATE, kernel, new Point( 0, 0 ), 4 );

		Core.inRange( temp, holeLowerBound, holeUpperBound, frame );

//		List<MatOfPoint> contours = new ArrayList<>( );
//		Mat hierarchy = new Mat( );
//		Imgproc.findContours( temp, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );
//
//		holeRects.clear();
//		for( int i = 0; i < contours.size( ); i++ ) {
//			MatOfPoint point = contours.get( i );
//			Rect boundingRect = Imgproc.boundingRect( point );
//			holeRects.add( new Rect(boundingRect.x + rect.x, boundingRect.y + rect.y,
//					boundingRect.width, boundingRect.height ) );
//		}

		return frame;
	}

	@Override
	public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
							 float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
		Paint paint = new Paint( );
		paint.setColor( Color.GREEN );
		paint.setStyle( Paint.Style.STROKE );
		paint.setStrokeWidth( scaleCanvasDensity * 4 );

		android.graphics.Rect rectangle = makeGraphicsRect( rect, scaleBmpPxToCanvasPx );
		canvas.drawRect( rectangle, paint );

		int minSize = 0;
		int maxSize = 2000;

		for( int i = 0; i < holeRects.size( ); i++ ) {
			if (holeRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect( holeRects.get( i ), scaleBmpPxToCanvasPx );
				int height = rect.height();
				int width = rect.width();
				int area = height * width;
				if (area < maxSize && area > minSize/* && Math.abs( 1 - (double)height / width) < 0.5 */ )
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
