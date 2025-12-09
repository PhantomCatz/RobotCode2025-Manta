package frc.robot.CatzAbstractions.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Bases.MotorIOInputsAutoLogged;
import frc.robot.CatzAbstractions.io.GenericMotorIO;

public abstract class GenericMotorSubsystem extends SubsystemBase {
    protected final GenericMotorIO io;
	protected final String name;

	protected final MotorIOInputsAutoLogged inputs[];

    public GenericMotorSubsystem(GenericMotorIO io, String name) {
		super(name);
		this.io = io; 

		this.name = name;
		this.inputs = new MotorIOInputsAutoLogged[io.getNumMotors()];
	}

    @Override
	public void periodic() {
		for (int i = 0; i < io.getNumMotors(); i++) {
			io.updateInputs(inputs[i]);
			Logger.processInputs(name + "[" + i + "]", inputs[i]);
		}
	}

	



}
