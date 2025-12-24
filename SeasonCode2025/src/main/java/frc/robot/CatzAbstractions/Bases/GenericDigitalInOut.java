package frc.robot.CatzAbstractions.Bases;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public abstract class GenericDigitalInOut {
	private final Debouncer debouncer;
	private final String name;

	public GenericDigitalInOut(Time debounce, String name) {
		debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
		this.name = name;
	}

	public abstract boolean get();

	public boolean getInverted() {
		return !get();
	}

	public boolean getDebounced() {
		return debouncer.calculate(get());
	}

	public boolean getDebouncedIfReal() {
		return Robot.isReal() && getDebounced();
	}

	public Command stateWait(boolean state) {
		return Commands.waitUntil(() -> get() == state);
	}

	public Command stateWaitWithDebounce(boolean state) {
		return Commands.waitUntil(() -> getDebounced() == state);
	}

	public Command stateWaitIfReal(boolean state, double waitSecondsSim) {
		return Commands.either(stateWait(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
	}

	public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim) {
		return Commands.either(
				stateWaitWithDebounce(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
	}



}
