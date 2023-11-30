package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
	public final static double SQAURE_THRESHOLD = 1.5;
	public final static double SIZE_THRESHOLD_LOW = 12;
	public final static double SIZE_THRESHOLD_HIGH = 30;



	PixelColor color = PixelColor.GREEN;
	public Scalar holeLowerBound = new Scalar( 0, 0, 0 );
	public Scalar holeUpperBound = new Scalar( 255, 255, 120 );
	AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
			.setDrawAxes(true)
			.setDrawCubeProjection(true)
			.setDrawTagOutline(true)
			.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
			.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
			.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
	Mat temp = new Mat( );

	Mat kernel = Mat.ones( 3, 3, CvType.CV_32F );

	ArrayList<Rect> holeRects = new ArrayList<>( );
	ArrayList<Rect> aprilRects = new ArrayList<>( );

	@Override
	public void init( int width, int height, CameraCalibration calibration ) {
	}

	@Override
	public Object processFrame( Mat frame, long captureTimeNanos ) {
		Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

		Core.inRange( temp, holeLowerBound, holeUpperBound, temp );

		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_ERODE, kernel, new Point( 0, 0 ), 3 );
		Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_DILATE, kernel, new Point( 0, 0 ), 3 );


		List<MatOfPoint> contours = new ArrayList<>( );
		Mat hierarchy = new Mat( );
		Imgproc.findContours( temp, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

		holeRects.clear();
		int highestY=0;
		int lowestY=1000000;
		for( int i = 0; i < contours.size( ); i++ ) {
			MatOfPoint point = contours.get( i );
			Rect boundingRect = Imgproc.boundingRect( point );
			if(boundingRect.height< boundingRect.width*SQAURE_THRESHOLD && boundingRect.width< boundingRect.height*SQAURE_THRESHOLD) {
				if(boundingRect.height<SIZE_THRESHOLD_HIGH && boundingRect.height>SIZE_THRESHOLD_LOW ) {
					if(boundingRect.width<SIZE_THRESHOLD_HIGH && boundingRect.width>SIZE_THRESHOLD_LOW ) {
						holeRects.add(boundingRect);
						if(boundingRect.y>highestY) {
							highestY= boundingRect.y;
						}
						if(boundingRect.y<lowestY) {
							lowestY= boundingRect.y;
						}
					}

				}

			}

		}
		ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
		int startingX = 0;
		int scaleX = 0;
		if(detections.size()>1) {
			scaleX = (int) (detections.get(1).center.x-detections.get(0).center.x);
		}
		for( int i=0; i>detections.size(); i++) {
			if(detections.get(i).id==4 || detections.get(i).id==1) {
				startingX= (int) detections.get(i).center.x-scaleX;
			}

		}
		for(int i=3; i>0; i--) {
			aprilRects.add(pointToPointRect(startingX,startingX+scaleX*2,lowestY,highestY));
			startingX+=scaleX;
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


		for( int i = 0; i < holeRects.size( ); i++ ) {
			if (holeRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect(holeRects.get(i), scaleBmpPxToCanvasPx);
				int height = rect.height();
				int width = rect.width();
				int area = height * width;
				canvas.drawRect(rect, paint);
			}
		}
		Paint paint2 = new Paint( );
		paint2.setColor( Color.BLUE );
		paint2.setStyle( Paint.Style.STROKE );
		paint2.setStrokeWidth( scaleCanvasDensity * 4 );
		for( int i = 0; i < aprilRects.size( ); i++ ) {
			if (aprilRects.get(i) != null) {
				android.graphics.Rect rect = makeGraphicsRect(aprilRects.get(i), scaleBmpPxToCanvasPx);
				int height = rect.height();
				int width = rect.width();
				int area = height * width;
				canvas.drawRect(rect, paint2);
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
	private Rect pointToPointRect(int x1, int x2,int y1,int y2) {
		return new Rect(x1,y1,x2-x1,y2-y1);
	}


}
