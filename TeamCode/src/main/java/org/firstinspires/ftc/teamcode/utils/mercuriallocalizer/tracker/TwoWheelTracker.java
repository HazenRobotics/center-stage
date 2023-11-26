package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker;

import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.Units;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.Angle;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.hardware.Encoder;

@SuppressWarnings("unused")
public class TwoWheelTracker extends WheeledTracker {
	private final Encoder left, middle;
	private final HeadingSupplier headingSupplier;
	private Angle currentTheta, previousTheta;
	private double deltaLeft, deltaMiddle, deltaTheta;

	public TwoWheelTracker(Pose2D initialPose, WheeledTrackerConstants.TwoWheeledTrackerConstants trackerConstants, Encoder left, Encoder middle, @NotNull HeadingSupplier headingSupplier) {
		super(initialPose, trackerConstants);
		this.left = left;
		this.middle = middle;
		this.headingSupplier = headingSupplier;
		this.currentTheta = initialPose.getTheta();
		this.previousTheta = initialPose.getTheta();
		setInsistFrequency(1);
		// sets the imu heading to the initial pose heading
		resetHeading(initialPose.getTheta());
	}

	/**
	 * called once per cycle, to prevent making too many calls to an encoder, etc
	 */
	@Override
	protected void updateValues() {
		left.updateVelocity();
		middle.updateVelocity();
		headingSupplier.updateHeading();

		WheeledTrackerConstants trackerConstants = getTrackerConstants();

		currentTheta = headingSupplier.getHeading();

		deltaLeft = trackerConstants.getLeftTicksConverter().toUnits(left.getVelocityDataPacket().getDeltaPosition(), Units.MILLIMETER);
		deltaMiddle = trackerConstants.getMiddleTicksConverter().toUnits(middle.getVelocityDataPacket().getDeltaPosition(), Units.MILLIMETER);
		deltaTheta = previousTheta.findShortestDistance(currentTheta);

		previousTheta = headingSupplier.getHeading();
	}

	/**
	 * @return the change in center displacement in millimeters
	 */
	@Override
	protected double findDeltaY() {
		return deltaLeft + (getTrackerConstants().getCenterOfRotationOffset().getY() * findDeltaTheta());
	}

	/**
	 * @return the change in horizontal displacement with correction for forward offset in millimeters
	 */
	@Override
	protected double findDeltaX() {
		return deltaMiddle + (getTrackerConstants().getCenterOfRotationOffset().getX() * findDeltaTheta());
	}

	@Override
	public void reset() {
		super.reset();
		left.reset();
		middle.reset();
		resetHeading(super.getInitialPose2D().getTheta());
	}

	/**
	 * @return the change in heading in radians
	 */
	@Override
	protected double findDeltaTheta() {
		return deltaTheta;
	}

	@Override
	public void resetHeading() {
		headingSupplier.resetHeading();
	}

	@Override
	public void resetHeading(Angle heading) {
		headingSupplier.resetHeading(heading);
	}

	/**
	 * enforce certain measurements, if an external measurement can be relied upon, gets automatically run every insist frequency cycles
	 */
	@Override
	protected void insist() {
		Pose2D currentPose = getPose2D();
		if (currentPose.getTheta().getRadians() != currentTheta.getRadians()) {
			setPose2D(new Pose2D(currentPose.getX(), currentPose.getY(), currentTheta));
		}
	}
}
