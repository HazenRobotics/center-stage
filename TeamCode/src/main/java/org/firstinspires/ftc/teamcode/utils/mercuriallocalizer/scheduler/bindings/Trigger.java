package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.bindings;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.Scheduler;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands.Command;

import java.util.function.BooleanSupplier;

@SuppressWarnings("unused")
public class Trigger {
	private final BooleanSupplier triggerCondition;
	private Command toRun;

	public Trigger(BooleanSupplier triggerCondition) {
		this.triggerCondition = triggerCondition;
		Scheduler.getSchedulerInstance().registerTrigger(this);
	}

	public Trigger(BooleanSupplier triggerCondition, Command toRun) {
		this.triggerCondition = triggerCondition;
		this.toRun = toRun;
		Scheduler.getSchedulerInstance().registerTrigger(this);
	}

	public BooleanSupplier getTriggerCondition() {
		return triggerCondition;
	}

	public void poll() {
		if (toRun != null && triggerCondition.getAsBoolean()) toRun.queue();
	}

	/**
	 * @param triggerCommand the new command to run when this trigger polls true
	 * @return self, for chaining
	 */
	public Trigger setCommand(Command triggerCommand) {
		this.toRun = triggerCommand;
		return this;
	}
}