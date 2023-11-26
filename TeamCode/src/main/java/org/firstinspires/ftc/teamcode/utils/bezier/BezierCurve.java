package org.firstinspires.ftc.teamcode.utils.bezier;

import org.opencv.core.Point;

public class BezierCurve {

	Point start;
	Point end;
	Point quadraticAugmenter;
	Point cubicAugmenter;

	public BezierCurve( Point start, Point end, Point quadraticAugmenter) {
		this.start = start;
		this.end = end;
		this.quadraticAugmenter = quadraticAugmenter;
	}

	public BezierCurve( Point start, Point end, Point quadraticAugmenter, Point cubicAugmenter) {
		this.start = start;
		this.end = end;
		this.quadraticAugmenter = quadraticAugmenter;
		this.cubicAugmenter = cubicAugmenter;
	}

	public Point quadraticPointAtT(double t) {
		double x = ( Math.pow( ( 1 - t), 2 ) * start.x ) +
				( 2 * ( 1 - t) * t * quadraticAugmenter.x ) +
				( Math.pow( t, 2 ) * end.x );

		double y = ( Math.pow( ( 1 - t), 2 ) * start.y ) +
				( 2 * ( 1 - t) * t * quadraticAugmenter.y ) +
				( Math.pow( t, 2 ) * end.y );

		return new Point( x, y );
	}

	public Point cubicPointAtT(double t) {
		double x = ( Math.pow( ( 1 - t), 3 ) * start.x ) +
				( 3 * Math.pow( ( 1 - t), 2 ) * t * quadraticAugmenter.x ) +
				( 3 * (1 - t) * Math.pow( t, 2 )  * cubicAugmenter.x ) +
				( Math.pow( t, 3 ) * end.x );

		double y = ( Math.pow( ( 1 - t), 3 ) * start.y ) +
				( 3 * Math.pow( ( 1 - t), 2 ) * t * quadraticAugmenter.y ) +
				( 3 * (1 - t) * Math.pow( t, 2 )  * cubicAugmenter.y ) +
				( Math.pow( t, 3 ) * end.y );

		return new Point( x, y );
	}
}
