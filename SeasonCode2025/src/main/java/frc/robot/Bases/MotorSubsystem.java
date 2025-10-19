package frc.robot.Bases;


import edu.wpi.first.units.measure.Angle;
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
		System.out.println("it worked!!!! base");
	}

}