package org.firstinspires.ftc.teamcode.utils.GVF;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GVFPath {
    public static double CORRECTION_DISTANCE = 30;
    public static double SAVING_THROW_DISTANCE = 100;
    public static double CURVATURE_CORRECTION_FACTOR = 1;
    public static int LOOKAHEAD_AMOUNT = 5;
    public static int SAMPLE_RATE = 100;

    CubicBezierCurve curve;
    Vector2[] TPointInterpolation;
    Vector2[] TDerivativeInterpolation;
    double[] TCurvatureInterpolation;
    Vector2[] DistanceToPointInterpolation;
    double[] TDistanceInterpolation;

    boolean isContinuous;

    double waitTime;

    public enum PathState {
        FOLLOW_PATH,
        USE_PID,
        DONE
    }

    public GVFPath(CubicBezierCurve curve) {
        this( curve, 0 );
        isContinuous = true;
    }

    public GVFPath(CubicBezierCurve curve, double waitTimeSeconds) {
        this.curve = curve;
        isContinuous = false;
        waitTime = waitTimeSeconds;
        TPointInterpolation = new Vector2[100];
        TDerivativeInterpolation = new Vector2[100];
        TCurvatureInterpolation = new double[100];
        DistanceToPointInterpolation = new Vector2[ (int) curve.getTotalCurveLength() + 1];
        interpolateCurve();
    }
    public Vector2 calculateGuidanceVector(Vector2 currentLocation) {

        int closestI = findClosestPointIndex(currentLocation);
        Vector2 closestPoint = TPointInterpolation[closestI];
        Vector2 curveDerivative = TDerivativeInterpolation[closestI];
        Vector2 robotToClosestPoint = closestPoint.subtract(currentLocation);
        double speedCurvatureMultiplier = 1 - (TCurvatureInterpolation[Math.min(closestI + LOOKAHEAD_AMOUNT, 99)] / CURVATURE_CORRECTION_FACTOR );

//        Vector2 endPoint = getEndPoint();

//        double directPursuitThreshold = SAMPLE_RATE - 1;
//
//        for (int i = SAMPLE_RATE - 1; i >= 0; i--) {
//            double dist = endPoint.subtract(curvePointInterpolation[i]).getMagSq();
//            if (dist > SAVING_THROW_DISTANCE) {
//                directPursuitThreshold = i;
//                break;
//            }
//        }

        double correctionFactor = robotToClosestPoint.getMagnitude() / CORRECTION_DISTANCE;

        correctionFactor = Math.min(1, correctionFactor);

        double movementDirection = hlerp(curveDerivative.getHeading(), robotToClosestPoint.getHeading(), correctionFactor);

//        if ((closestI == 100 && Math.abs(currentLocation.subtract(closestPoint).getHeading() - curveDerivative.getHeading()) <= 0.5 * Math.PI) ||
//            closestI >= directPursuitThreshold) {
//            movementDirection = endPoint.subtract(currentLocation).getHeading();
//        }

        return new Vector2(Math.cos(movementDirection), Math.sin(movementDirection)).scalarMultiply( speedCurvatureMultiplier );
    }

    /*
    Hlerp - heading lerp.
    Interpolates between a and b, taking the shortest path across the range
    [-pi, pi] assuming the input range is continuous across said range.

    Say a = -0.9pi, b = 0.9pi. traditional lerp would rotate counterclockwise,
    passing through 0 at t = 0.5. Hlerp will rotate clockwise, passing through
    +/- pi at t = 0.5.

    Hlerp is used to avoid an edge case where the movement direction
    passes through +/- pi as it transitions from to-path correction to
    on-path guidance.
     */
    public double hlerp(double a, double b, double t) {
        double diff = b - a;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return a + t * diff;
    }

    private double lerp(double a, double b, double t) {
        return (1 - t) * a + t * b;
    }

    public int findClosestPointIndex(Vector2 currentPosition) {
        //long startTime = System.nanoTime();
        int minI = -1;
        double minDist = Double.POSITIVE_INFINITY;
        int SAMPLE_DENSITY = 100;

        for( int i = 0; i < SAMPLE_DENSITY; i++ ) {
            double dist = TPointInterpolation[i].subtract( currentPosition ).getMagSq( );
            if( dist < minDist ) {
                minDist = dist;
                minI = i;
            }
        }
        //System.out.println(String.format("Calculated closest point in %.3f ms", (System.nanoTime() - startTime) / 1e6));
        return minI;
    }

    public void interpolateCurve() {
        for (int i = 0; i < SAMPLE_RATE; i++) {
            double t = i / (double) SAMPLE_RATE;
            TPointInterpolation[i] = curve.calculate( t );
            TDerivativeInterpolation[i] = curve.firstDerivative( t );
            TCurvatureInterpolation[i] = curve.getCurvature( t );
//            TDistanceInterpolation[i] = curve.getCurveLength( 0, t );
        }

        double t = 0;
        int nextGoalLength = 0;
        double length = 0;

        while (nextGoalLength < curve.getTotalCurveLength()) {
            length += curve.firstDerivative( t ).getMagnitude() * 0.001;

            if(length > nextGoalLength) {
                System.out.println( "length : " + length + " t: " + t );
                DistanceToPointInterpolation[nextGoalLength] = curve.calculate( t );
                nextGoalLength++;
            }
            t += 0.001;
        }
    }

    public PathState evaluateState( Vector2 currentLocation ) {

        double robotToEndDist = getDistanceFromEnd( currentLocation );

        if (robotToEndDist < 1 || (robotToEndDist < 5 && isContinuous)) return PathState.DONE;
        else if (isContinuous) return PathState.FOLLOW_PATH;
        else if (robotToEndDist < 15) return PathState.USE_PID;
        else return PathState.FOLLOW_PATH;
    }

    public Vector2 getEndPoint() {
        return TPointInterpolation[SAMPLE_RATE - 1];
    }

    public CubicBezierCurve getCurve() {
        return curve;
    }

    public boolean isContinuous( ) {
        return isContinuous;
    }

    public void setContinuous( boolean continuous ) {
        isContinuous = continuous;
    }

    public double getWaitTime( ) {
        return waitTime;
    }

    public double getDistanceFromEnd( Vector2 currentLocation ) {
        return getEndPoint().subtract(currentLocation).getMagnitude();
    }

    public Vector2[] getTPointInterpolation() {
        return TPointInterpolation;
    }

    public Vector2[] getDistanceToPointInterpolation() {
        return DistanceToPointInterpolation;
    }

//    public Vector2D getCentripetalForceAtPoint( double curvature ) {
//        Vector2D velocityVector = tracker.getDeltaPositionVector().scalarMultiply( currentHz );
//
//        return velocityVector.vectorMultiply( velocityVector ).scalarMultiply( robotMass ).scalarMultiply( curvature );
//    }
}
