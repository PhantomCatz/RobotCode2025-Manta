package frc.robot.Bases;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    protected final MotorIO io;
	protected final String name;

    public MotorSubsystem(MotorIO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

    @Override
	public void periodic() {
		io.updateInputs(io.getMotorIOInputs());
		System.out.println("it worked!!!! base");
	}

}