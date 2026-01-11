package frc.robot.CatzAbstractions.Bases;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzAbstractions.io.GenericMotorIO;

public abstract class GenericMotorSubsystem<S extends GenericMotorIO<I>, I extends GenericMotorIO.MotorIOInputs> extends SubsystemBase {
	protected final S io;
	protected final I inputs;
	protected final String name;

	public GenericMotorSubsystem(S io, I inputs, String name) {
		super(name);
		this.io = io;
		this.inputs = inputs;
		this.name = name;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, (LoggableInputs) inputs);
	}

	public void setDutyCycle(double dutyCycle) {
		io.setDutyCycleSetpoint(dutyCycle);
	}

	public Command setDutyCycleCommand(double dutyCycle) {
		return runOnce(() -> setDutyCycle(dutyCycle));
	}

	public double getVelocityRPS() {
		return inputs.velocityRPS;
	}

	public double getPosition() {
		return inputs.position;
	}

	public double[] getSupplyCurrent() {
		return inputs.supplyCurrentAmps;
	}

	public double getAcceleration() {
		return inputs.accelerationRPS;
	}

	public double[] getTemp() {
		return inputs.tempCelcius;
	}

	public double[] getAppliedVoltage() {
		return inputs.appliedVolts;
	}
}
