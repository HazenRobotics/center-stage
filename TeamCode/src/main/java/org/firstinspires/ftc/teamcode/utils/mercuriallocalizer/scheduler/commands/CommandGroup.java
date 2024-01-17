package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands;

import java.util.Arrays;
import java.util.Collection;

@SuppressWarnings("unused")
public interface CommandGroup extends Command {
	default CommandGroup addCommands(Command... commands) {
		return addCommands(Arrays.asList(commands));
	}

	CommandGroup addCommands(Collection<Command> commands);
}
