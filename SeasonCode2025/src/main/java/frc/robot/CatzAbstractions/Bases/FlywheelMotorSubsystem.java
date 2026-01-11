package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.DelayedBoolean;
import frc.robot.Utilities.Setpoint;

public abstract class FlywheelMotorSubsystem<S extends GenericMotorIO<I>, I extends GenericMotorIO.MotorIOInputs> extends GenericMotorSubsystem<S, I> {

	protected final S io;
	protected final I inputs;
	protected final String name;
	protected final double epsilonThreshold;
	private Setpoint setpoint;

	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	public FlywheelMotorSubsystem(S io, I inputs, String name, double epsilonThreshold) {
		super(io, inputs, name);
		this.inputs = inputs;
		this.io = io;
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
	}


	@Override
	public void periodic() {
		super.periodic();
	}




}
