package frc.robot.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
		io.updateInputs(inputs);
		Logger.processInputs("RealInputs/"+name, inputs);
		//System.out.println(inputs.absoluteEncoderPositionRads);
		// System.out.println("it worked!!!! base");

	}

	//NOTE this file should have get methods that returns all values from MotorIOInputsAutoLogged
	//also have methods that set the setpoint of the motor. 

}
