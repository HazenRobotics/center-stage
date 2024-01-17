package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.subsystems;

import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.OpModeEX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands.Command;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands.LambdaCommand;

@SuppressWarnings("unused")
public abstract class Subsystem implements SubsystemInterface {
	public final OpModeEX opModeEX;
	private Command defaultCommand;

	public Subsystem(@NotNull OpModeEX opModeEX) {
		this.opModeEX = opModeEX;
		setDefaultCommand(new LambdaCommand().setRequirements(this).setInterruptible(true).setExecute(this::defaultCommandExecute).setFinish(() -> false));
		opModeEX.getScheduler().registerSubsystem(this);
	}

	public Command getDefaultCommand() {
		return defaultCommand;
	}

	@Override
	public void setDefaultCommand(Command defaultCommand) {
		this.defaultCommand = defaultCommand;
	}

	/**
	 * @return if currently required by a non-default command
	 */
	public final boolean isBusy() {
		return opModeEX.getScheduler().isBusy(this);
	}

}
