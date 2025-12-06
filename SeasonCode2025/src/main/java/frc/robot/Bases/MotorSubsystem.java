package frc.robot.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Bases.MotorIO.Setpoint;

public class MotorSubsystem extends SubsystemBase {
    protected final MotorIO io;
	protected final String name;


    public MotorSubsystem(MotorIO io, String name) {
		super(name);
		this.io = io; // This is null when you run it TODO

		this.name = name;
	}


    @Override
	public void periodic() {
		io.updateInputs();
		//System.out.println(inputs.absoluteEncoderPositionRads);
		// System.out.println("it worked!!!! base");

	}

	public void runSetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public double getVelocityInch() {
		return io.getVelocityInch();
	}

	public double getPositionInch() {
		return io.getPositionInch();
	}

	public double getSupplyCurrent() {
		return io.getSupplyCurrent();
	}

	public double getAcceleration() {
		return io.getAcceleration();
	}

	public double getTemp() {
		return io.getTemp();
	}

	public double getAppliedVoltage() {
		return io.getAppliedVoltage();
	}

	public double getRotations() {
		return io.getRotations();
	}

	// public double motorRotations = 0.0;
	// public double absoluteEncoderPositionRads = 0.0;
	// public double relativeEncoderPositionRads = 0.0;
	// public double velocityInchPerSec = 0.0;
	// public double acceleration = 0.0;
	// public double supplyCurrentAmps = 0.0;
	// public double torqueCurrentAmps = 0.0;
	// public double appliedVoltage = 0.0;
	// public double tempCelcius = 0.0;

	//NOTE this file should have get methods that returns all values from MotorIOInputsAutoLogged

}
