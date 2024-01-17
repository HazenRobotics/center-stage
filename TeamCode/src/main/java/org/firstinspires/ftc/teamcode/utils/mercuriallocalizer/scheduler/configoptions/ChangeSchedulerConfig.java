package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.configoptions;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.OpModeEX;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.Scheduler;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.commands.LambdaCommand;

import java.io.IOException;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

@TeleOp(name = "Edit Scheduler Config Options", group = "?") // we use '?' to move it to the bottom of the list
@Disabled
public class ChangeSchedulerConfig extends OpModeEX {
	private int selection, selectionSize;
	private String selectionString;

	@Override
	public void registerSubsystems() {

	}

	@Override
	public void initEX() {
	}

	@Override
	public void registerBindings() {
		gamepadEX1().dpad_up().onTrue(
				new LambdaCommand().setInit(() -> {
					selection++;
					if (selection < 0) selection += selectionSize;
					selection %= selectionSize;
				})
		);
		gamepadEX1().dpad_down().onTrue(
				new LambdaCommand().setInit(() -> {
					selection--;
					if (selection < 0) selection += selectionSize;
					selection %= selectionSize;
				})
		);
		gamepadEX1().a().onTrue(
				new LambdaCommand().setInit(() -> {
					Scheduler.getConfigOptionsManager().updateValue(selectionString, Boolean.FALSE.equals(Scheduler.getConfigOptionsManager().getTomlParseResult().getBoolean(selectionString)));
					try {
						Scheduler.getConfigOptionsManager().update();
					} catch (IOException e) {
						throw new RuntimeException("failed to update settings: \n" + e);
					}
				})
		);
	}

	@Override
	public void init_loopEX() {

	}

	@Override
	public void startEX() {

	}

	@Override
	public void loopEX() {
		Set<Map.Entry<String, Object>> config = Scheduler.getConfigOptionsManager().getTomlParseResult().dottedEntrySet();
		selectionSize = config.size();

		telemetry.addLine("Use up and down on gamepad1's dpad to select the setting you want to change, then press a to change it");
		telemetry.addLine();
		telemetry.addLine("Current Settings");

		Iterator<Map.Entry<String, Object>> configIterator = config.iterator();
		for (int i = 0; configIterator.hasNext(); i++) {
			Map.Entry<String, Object> configEntry = configIterator.next();

			if (selection == i) {
				telemetry.addData(configEntry.getKey(), configEntry.getValue() + " <--");
				selectionString = configEntry.getKey();
			} else {
				telemetry.addData(configEntry.getKey(), configEntry.getValue());
			}
		}
	}

	@Override
	public void stopEX() {

	}
}
