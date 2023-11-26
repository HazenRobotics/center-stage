package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.encoderticksconverter.Units;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.Angle;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

import java.util.Locale;

/**
 * represents a robot position through a global x and y coordinate (always millimeters internally) and an angle (always radians internally) angles are represented as starting at the positive x-axis and increasing in the anticlockwise direction
 */
@SuppressWarnings("unused")
public class Pose2D {
	private final double x;
	private final double y;
	private final AngleRadians theta;


	/**
	 * constructs a pose
	 *
	 * @param x     the robot's x position
	 * @param y     the robot's y position
	 * @param theta the robot's angle
	 * @param units the units used for x and y
	 */
	public Pose2D(double x, double y, @NotNull Angle theta, @NotNull Units units) {
		this.x = units.toMillimeters(x);
		this.y = units.toMillimeters(y);
		this.theta = theta.toAngleRadians();
	}

	/**
	 * constructs a pose, assumes the use of degrees
	 *
	 * @param x     the robot's x position
	 * @param y     the robot's y position
	 * @param theta the robot's angle, in degrees
	 * @param units the units used for x and y
	 */
	public Pose2D(double x, double y, double theta, Units units) {
		this(x, y, new AngleDegrees(theta), units);
	}

	/**
	 * constructs a pose, assumes the use of millimeters
	 *
	 * @param x     the robot's x position
	 * @param y     the robot's y position
	 * @param theta the robot's angle
	 */
	public Pose2D(double x, double y, Angle theta) {
		this(x, y, theta, Units.MILLIMETER);
	}

	/**
	 * constructs a pose, assumes the use of millimeters and degrees
	 *
	 * @param x     the robot's x position
	 * @param y     the robot's y position
	 * @param theta the robot's angle, in degrees
	 */
	public Pose2D(double x, double y, double theta) {
		this(x, y, new AngleDegrees(theta), Units.MILLIMETER);
	}

	/**
	 * a default constructor that sets everything to 0
	 */
	public Pose2D() {
		this(0, 0, 0, Units.MILLIMETER);
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public Angle getTheta() {
		return theta;
	}

	/**
	 * non mutating
	 *
	 * @param other
	 * @return a new pose with the transformation applied to it
	 */
	public Pose2D add(@NotNull Pose2D other) {
		return new Pose2D(x + other.getX(), y + other.getY(), theta.add(other.getTheta()));
	}

	/**
	 * non mutating
	 *
	 * @param x
	 * @param y
	 * @param theta
	 * @return a new pose with the transformation applied to it
	 */
	public Pose2D add(double x, double y, Angle theta) {
		return new Pose2D(this.x + x, this.y + y, this.theta.add(theta));
	}

	/**
	 * non mutating
	 *
	 * @param other
	 * @return a new pose with the transformation applied to it
	 */
	public Pose2D subtract(@NotNull Pose2D other) {
		return new Pose2D(x - other.getX(), y - other.getY(), theta.subtract(other.getTheta()));
	}

	/**
	 * non mutating
	 *
	 * @param x
	 * @param y
	 * @param theta
	 * @return a new pose with the transformation applied to it
	 */
	public Pose2D subtract(double x, double y, Angle theta) {
		return new Pose2D(this.x - x, this.y - y, this.theta.subtract(theta));
	}

	public Vector2D toVector2D() {
		return new Vector2D(this.getX(), this.getY());
	}

	@Override
	public boolean equals(@Nullable @org.jetbrains.annotations.Nullable Object obj) {
		if (!(obj instanceof Pose2D)) return false;
		Pose2D other = (Pose2D) obj;
		return this.getX() == other.getX() && this.getY() == other.getY() && this.getTheta().equals(other.getTheta());
	}

	@NonNull
	@NotNull
	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "x: %f, y: %f, theta: %s", getX(), getY(), getTheta().toString());
	}
}
