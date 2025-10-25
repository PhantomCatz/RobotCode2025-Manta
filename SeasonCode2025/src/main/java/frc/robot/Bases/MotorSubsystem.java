package frc.robot.Bases;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    protected final MotorIO io;
	protected final String name;

	protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

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

}
