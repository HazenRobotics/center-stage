package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.Units;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

public class ThreeWheelTracker extends WheeledTracker {
	private final Encoder left, right, middle;
	private double deltaLeft, deltaRight, deltaMiddle, deltaTheta;

	public ThreeWheelTracker( Pose2D initialPose, WheeledTrackerConstants.ThreeWheeledTrackerConstants trackerConstants, Encoder left, Encoder right, Encoder middle) {
		super(initialPose, trackerConstants);
		this.left = left;
		this.right = right;
		this.middle = middle;
	}

	/**
	 * called once per cycle, to prevent making too many calls to an encoder, etc
	 */
	@Override
	protected void updateValues() {
		left.updateVelocity();
		right.updateVelocity();
		middle.updateVelocity();

		WheeledTrackerConstants trackerConstants = getTrackerConstants();

		deltaLeft = trackerConstants.getLeftTicksConverter().toUnits(left.getVelocityDataPacket().getDeltaPosition(), Units.MILLIMETER);
		deltaRight = trackerConstants.getRightTicksConverter().toUnits(right.getVelocityDataPacket().getDeltaPosition(), Units.MILLIMETER);
		deltaMiddle = trackerConstants.getMiddleTicksConverter().toUnits(middle.getVelocityDataPacket().getDeltaPosition(), Units.MILLIMETER);
	}

	/**
	 * @return the change in center displacement in millimeters
	 */
	@Override
	protected double findDeltaY() {
		return (deltaLeft + deltaRight) / 2;
	}

	/**
	 * @return the change in horizontal displacement with correction for forward offset in millimeters
	 */
	@Override
	protected double findDeltaX() {
		return deltaMiddle + (getTrackerConstants().getCenterOfRotationOffset().getX() * findDeltaTheta());
	}

	/**
	 * @return the change in heading in radians
	 */
	@Override
	protected double findDeltaTheta() {
		deltaTheta = (deltaRight - deltaLeft) / ((WheeledTrackerConstants.ThreeWheeledTrackerConstants) getTrackerConstants()).getTrackWidth();
		return deltaTheta;
	}

	public double getDeltaTheta() {
		return deltaTheta;
	}

	@Override
	public void reset() {
		super.reset();
		left.reset();
		right.reset();
		middle.reset();
	}

	/**
	 * enforce certain measurements, if an external measurement can be relied upon, gets automatically run every insist frequency cycles
	 */
	@Override
	protected void insist() {

	}
}
