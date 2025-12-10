package frc.robot.CatzAbstractions.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Bases.MotorIOInputsAutoLogged;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.Setpoint;

public abstract class GenericMotorSubsystem extends SubsystemBase {
	protected final GenericMotorIO io;
	protected final String name;

	protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

	public GenericMotorSubsystem(GenericMotorIO io, String name) {
		super(name);
		this.io = io; 

		this.name = name;
	}

	@Override
	public final void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);
		customGenericPeriodic();
	}

	
	public void customGenericPeriodic() {

	}



	public void runSetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public double getVelocityRPS() {
		return inputs.velocityRPS;
	}

	public double getPosition() {
		return inputs.relativeEncoderPosition;
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
