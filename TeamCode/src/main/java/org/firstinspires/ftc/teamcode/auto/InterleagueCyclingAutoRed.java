package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;

public class InterleagueCyclingAutoRed extends LinearOpMode {

	enum AutoStates {
		SPIKE_MARK_SCANNING,
		SCORE_PURPLE,
		PICKUP_TOP_STACK,
		SCORE_YELLOW,
		CYCLE,
		PARK
	}

	enum SpikeMarkStates {
		DRIVE_TO_MARK,
		SPIT_OUT_PURPLE,
	}

	enum PickupStackState {
		DRIVE_TO_STACK,
		GRAB_TOP_PIXEL
	}

	enum BackdropScoringStates {
		DRIVE_TO_BACKDROP,
		SCORE_YELLOW,
		SCORE_WHITE
	}
	enum CycleStates {
		DRIVE_TO_STACK,
		PICKUP_PIXELS,
		SCOOT_FORWARD,
		DRIVE_TO_BACKDROP,
		SCORE_BACKDROP
	}

	AutoStates autoState = AutoStates.SCORE_PURPLE;
	SpikeMarkStates spikeState = SpikeMarkStates.DRIVE_TO_MARK;
	PickupStackState pickupStackState = PickupStackState.DRIVE_TO_STACK;
	BackdropScoringStates backdropScoringStates = BackdropScoringStates.DRIVE_TO_BACKDROP;
	CycleStates cycleStates = CycleStates.DRIVE_TO_STACK
	RedPropProcessor.PropPosition position;
	@Override
	public void runOpMode( ) throws InterruptedException {

	}
}
