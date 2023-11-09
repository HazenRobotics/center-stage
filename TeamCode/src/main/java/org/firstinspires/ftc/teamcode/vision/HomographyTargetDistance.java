package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;
import Jama.Matrix;

/**
 * This class contains the variables and methods to be able to calculate a 3D position from a 2D image pixel.
 * Matrices need to be re-calculated if the camera is moved on the robot.
 */
public class HomographyTargetDistance {

	//Value from solvePnP
	private static final Matrix TRANSLATION_VECTOR = new Matrix( new double[][] {
			{-4.99441617},
			{8.30287095},
			{20.47677155}
	} );

	//Value after applying Rodriguez rotation formula to rotation vector from solvePnP
	private static final Matrix ROTATION_MATRIX = new Matrix( new double[][] {
			{0.99633589, 0.02140191,  -0.08280548},
			{0.02887255, 0.82718011,  0.56119465},
			{0.08050569,  -0.56152918,  0.82353131}
	} );

	//Value from camera calibration
	private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
			{882.06566201,   0.0,    733.53296718},
			{0.0,         349.92692564, 401.9873917},
			{0.0, 0.0, 1.0}
	} );
	//Z axis is always 1 since all the objects are on the ground
	final static double Z_CONST = 1;


	/**
	 * Calculates the position of the corresponding point in 3D space from a camera point
	 * @param point camera point at which to find the corresponding 3D point
	 * @return point's position relative to the camera
	 */
	public static double[] positionFromPoint( Point point ) {

		//Change point into a matrix
		Matrix pointMatrix = new Matrix( new double[][] {
				{point.x},
				{point.y},
				{1}
		} );

		//Calculating scalar value
		Matrix leftSideMatrix = ROTATION_MATRIX.inverse().times( CAMERA_MATRIX.inverse() ).times(pointMatrix);
		Matrix rightSideMatrix = ROTATION_MATRIX.inverse().times( TRANSLATION_VECTOR );
		double scalar = (Z_CONST + rightSideMatrix.get( 2, 0 )) / leftSideMatrix.get( 2, 0 );

		//(x,y) position in mm
		Matrix calculatedPosition = ROTATION_MATRIX.inverse().times( CAMERA_MATRIX.inverse().times( pointMatrix ).times( scalar ).minus( TRANSLATION_VECTOR ) );

		//Divide by 25.4 to get inches from mm
		calculatedPosition = calculatedPosition.times( 1 / 25.4 );

		//Swap X and Y, and add 24 to X to account for camera coordinate system
		double x = calculatedPosition.get( 0, 0 );
		double y = calculatedPosition.get( 1, 0 );
		return new double[]{y, x};
	}

}