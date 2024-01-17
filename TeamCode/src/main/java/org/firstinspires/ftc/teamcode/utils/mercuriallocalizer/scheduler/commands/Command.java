package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.OpModeEX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.Scheduler;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.subsystems.SubsystemInterface;

import java.util.Set;

public interface Command {
	/**
	 * Gets run once when a command is first scheduled
	 */
	void initialise();

	/**
	 * Gets run once per loop until {@link #finished()}
	 */
	void execute();

	/**
	 * Gets run once when {@link #finished()} or when interrupted
	 */
	void end(boolean interrupted);

	/**
	 * the supplier for the natural end condition of the command
	 *
	 * @return true when the command should finish
	 */
	boolean finished();

	/**
	 * @return the set of subsystems required by this command
	 */
	Set<SubsystemInterface> getRequiredSubsystems();

	/**
	 * @return the set of OpMode run states during which this command is allowed to run
	 */
	Set<OpModeEX.OpModeEXRunStates> getRunStates();

	/**
	 * @return if this command is allowed to be interrupted by others
	 */
	default boolean interruptable() {
		return true;
	}

	/**
	 * schedule the command with the scheduler
	 */
	default void queue() {
		if (!Scheduler.getSchedulerInstance().isScheduled(this)) Scheduler.getSchedulerInstance().scheduleCommand(this);
	}
}
