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

public class BoundingBoxProcessor implements VisionProcessor {
	public Scalar lowerBound = new Scalar( 13,119,0 );
	public Scalar upperBound = new Scalar( 29,255,255 );;

	Mat kernel = Mat.ones(3,3, CvType.CV_32F);
	Mat temp = new Mat();
	ArrayList<Rect> rects = new ArrayList<>();
	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		Core.inRange( temp, lowerBound, upperBound, temp );

		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_ERODE, kernel, new Point(0,0), 10  );
		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_DILATE, kernel, new Point(0,0), 10  );

		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(temp, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

		if (contours.size() > 0) {
			for( int i = 0; i < contours.size( ); i++ ) {
				MatOfPoint point = contours.get( i );
				Rect boundingRect = Imgproc.boundingRect( point );
				rects.add( boundingRect );
			}
		}

		return frame;
	}

	@Override
	public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
							 float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
		Paint selectedPaint = new Paint( );
		selectedPaint.setColor( Color.GREEN );
		selectedPaint.setStyle( Paint.Style.STROKE );
		selectedPaint.setStrokeWidth( scaleCanvasDensity * 4 );

		if( rects.size() > 0 ) {
			for( int i = 0; i < rects.size( ); i++ ) {
				android.graphics.Rect rect = makeGraphicsRect( rects.get( i ), scaleBmpPxToCanvasPx );
				canvas.drawRect( rect, selectedPaint );
				rects.remove( i-- );
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
