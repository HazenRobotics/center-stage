package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.tracker;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.Angle;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleRadians;

public interface HeadingSupplier {
	/**
	 * implementations are recommended to supply a {@link AngleRadians} if possible
	 *
	 * @return the current heading of the robot
	 */
	Angle getHeading();

	/**
	 * <p>can be called by consumers that need to ensure that an update is run regardless of if the implementation is self updating or not</p>
	 * may be left blank by implementation if updates are not required, or are handled differently.
	 */
	void updateHeading();

	/**
	 * resets the heading to 0 at this point
	 */
	void resetHeading();

	/**
	 * resets the heading to the supplied Angle at this point
	 */
	void resetHeading(Angle heading);
}
