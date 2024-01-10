package org.firstinspires.ftc.teamcode.utils.GVF;

public class CubicBezierCurve {

	private Vector2 p0, p1, p2, p3;
	double curveLength;

	public CubicBezierCurve( Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3 ) {
		this.p0 = p0;
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;

		curveLength = getCurveLength( 0, 1 );
	}

	public Vector2 getP0( ) {
		return p0;
	}

	public void setP0( Vector2 p0 ) {
		this.p0 = p0;
	}

	public Vector2 getP1( ) {
		return p1;
	}

	public void setP1( Vector2 p1 ) {
		this.p1 = p1;
	}

	public Vector2 getP2( ) {
		return p2;
	}

	public void setP2( Vector2 p2 ) {
		this.p2 = p2;
	}

	public Vector2 getP3( ) {
		return p3;
	}

	public void setP3( Vector2 p3 ) {
		this.p3 = p3;
	}

	public CubicBezierCurve createC1ContinuousCurve( Vector2 newP2, Vector2 newP3 ) {
		// Calculate the new control points (c2 and c3) for C1 continuity
		Vector2 c1 = p3.scalarMultiply( 2 );

		c1 = c1.subtract( p2 );

		// Return the new CubicBezierCurve with C1 continuity
		return new CubicBezierCurve( p3, c1, newP2, newP3 );
	}

	public Vector2 calculate( double t ) {
		// (1 - t)^3 * P0 + 3 * t * (1 - t)^2 * P1 + 3 * t^2 * (1 - t) * P2 + t^3 * P3
		double w = 1 - t;
		Vector2 firstTerm = p0.scalarMultiply( w * w * w );
		Vector2 secondTerm = p1.scalarMultiply( 3 * t * w * w );
		Vector2 thirdTerm = p2.scalarMultiply( 3 * t * t * w );
		Vector2 fourthTerm = p3.scalarMultiply( t * t * t );
		return firstTerm.add( secondTerm ).add( thirdTerm ).add( fourthTerm );
	}

	public Vector2 firstDerivative( double t ) {
		double w = 1 - t;
		Vector2 firstTerm = p1.subtract( p0 ).scalarMultiply( 3 * w * w );
		Vector2 secondTerm = p2.subtract( p1 ).scalarMultiply( 6 * w * t );
		Vector2 thirdTerm = p3.subtract( p2 ).scalarMultiply( 3 * t * t );
		return firstTerm.add( secondTerm ).add( thirdTerm );
	}

	public Vector2 secondDerivative( double t ) {
		double w = 1 - t;
		Vector2 firstTerm = p1.subtract( p0 ).scalarMultiply( -6 * w );
		Vector2 secondTerm = p2.subtract( p1 ).scalarMultiply( -6 * (2 * t - 1) );
		Vector2 thirdTerm = p3.subtract( p2 ).scalarMultiply( 6 * t );
		return firstTerm.add( secondTerm ).add( thirdTerm );
	}

	public double slope( double t ) {
		Vector2 dt = firstDerivative( t );
		return dt.getY( ) / dt.getX( );
	}

	public double heading( double t ) {
		return firstDerivative( t ).getHeading( );
	}

	public double getCurvature( double t ) {
		Vector2 dt = firstDerivative( t );
		Vector2 d2t = secondDerivative( t );

		return (dt.getX( ) * d2t.getY( ) - dt.getY( ) * d2t.getX( ))
				/ Math.pow( Math.pow( dt.getX( ), 2 ) + Math.pow( dt.getY( ), 2 ), 1.5 );
	}

	public double getR( double t ) {
		return 1 / getCurvature( t );
	}

	public double getCurveLength( double startT, double endT ) {
		Vector2 pointDerivative;
		double length = 0;

		for( double i = startT; i < endT; i += 0.001 ) {
			pointDerivative = firstDerivative( i );
			length += 0.001 * pointDerivative.getMagnitude();
		}
		return length;
	}

	public double getTotalCurveLength( ) {
		return curveLength;
	}
}
