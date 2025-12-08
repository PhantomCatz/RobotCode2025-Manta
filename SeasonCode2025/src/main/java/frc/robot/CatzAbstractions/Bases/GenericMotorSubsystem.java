package frc.robot.CatzAbstractions.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Bases.MotorIOInputsAutoLogged;
import frc.robot.CatzAbstractions.io.GenericMotorIO;

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
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);


	}



}
