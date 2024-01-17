package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.bindings.gamepadex.domainbindingbuilder;

import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scheduler.bindings.Binding;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@SuppressWarnings("unused")
public class DomainBinding<S extends DoubleSupplier> extends Binding<DomainBinding<S>> {
	private final S s;

	public DomainBinding(S s, BooleanSupplier internalInput) {
		super(internalInput);
		this.s = s;
	}

	public S endBinding() {
		return s;
	}
}
