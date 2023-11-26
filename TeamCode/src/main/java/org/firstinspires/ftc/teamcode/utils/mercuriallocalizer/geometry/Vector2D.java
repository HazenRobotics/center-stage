package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.Angle;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

import java.util.Locale;

@SuppressWarnings("unused")
public class Vector2D {
	private double x, y;

	public Vector2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * constructs a new default vector with values of 0 for both x and y
	 */
	public Vector2D() {
		this(0, 0);
	}

	/**
	 * The polar constructor for vectors, uses radians
	 *
	 * @param r magnitude
	 * @param t theta in radians
	 * @return a Vector2D with the above values assigned to x and y coordinates
	 */
	@NotNull
	@Contract("_, _ -> new")
	public static Vector2D fromPolar(double r, double t) {
		return fromPolar(r, new AngleRadians(t));
	}

	/**
	 * The polar constructor for vectors
	 *
	 * @param r magnitude
	 * @param t theta
	 * @return a Vector2D with the supplied properties
	 */
	@NotNull
	@Contract("_, _ -> new")
	public static Vector2D fromPolar(double r, @NotNull Angle t) {
		return new Vector2D(r * Math.cos(t.getRadians()), r * Math.sin(t.getRadians()));
	}

	public double getX() {
		return x;
	}

	/**
	 * mutates state
	 *
	 * @param x
	 */
	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	/**
	 * mutates state
	 *
	 * @param y
	 */
	public void setY(double y) {
		this.y = y;
	}

	public Angle getHeading() {
		return new AngleRadians(Math.atan2(y, x));
	}

	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	/**
	 * mutates state
	 *
	 * @param x
	 * @param y
	 * @return self
	 */
	public Vector2D set(double x, double y) {
		this.x = x;
		this.y = y;
		return this;
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D add(double x, double y) {
		return new Vector2D(this.x + x, this.y + y);
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D subtract(double x, double y) {
		return new Vector2D(this.x - x, this.y - y);
	}

	/**
	 * non-mutating
	 *
	 * @param other
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D add(@NotNull Vector2D other) {
		return this.add(other.x, other.y);
	}

	/**
	 * non-mutating
	 *
	 * @param other
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D subtract(@NotNull Vector2D other) {
		return this.subtract(other.x, other.y);
	}

	/**
	 * non-mutating
	 *
	 * @param factor the scalar multiplication
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D scalarMultiply(double factor) {
		return fromPolar(getMagnitude() * factor, getHeading());
	}

	/**
	 * dot product
	 *
	 * @param other
	 * @return
	 */
	public double dot(@NotNull Vector2D other) {
		return this.getX() * other.getX() + this.getY() * other.getY();
	}

	/**
	 * rotates anti-clockwise
	 *
	 * @param angle
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D rotate(@NotNull Angle angle) {
		double cos = Math.cos(angle.getRadians());
		double sin = Math.sin(angle.getRadians());
		return new Vector2D(cos * getX() - sin * getY(), sin * getX() + cos * getY());
	}

	/**
	 * non-mutating
	 *
	 * @return a new vector with the same heading but a magnitude of 1
	 */
	public Vector2D getUnitVector() {
		return fromPolar(1, this.getHeading());
	}

	@Override
	public boolean equals(@Nullable @org.jetbrains.annotations.Nullable Object obj) {
		if (!(obj instanceof Vector2D)) return false;
		Vector2D other = (Vector2D) obj;
		return this.getX() == other.getX() && this.getY() == other.getY();
	}

	@NonNull
	@NotNull
	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "x: %f, y: %f", getX(), getY());
	}
}
