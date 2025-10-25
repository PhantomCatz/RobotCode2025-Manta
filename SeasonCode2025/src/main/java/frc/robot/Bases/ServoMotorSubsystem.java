package frc.robot.Bases;

import frc.robot.Bases.MotorIO.Setpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoMotorSubsystem<IO extends MotorIO> extends SubsystemBase {
    protected final IO io;
	protected final String name;

	public ServoMotorSubsystem() {
		super();
		io = null;
		name = null;
	}

    public ServoMotorSubsystem(IO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

    @Override
	public void periodic() {
		io.updateInputs(io.getMotorIOInputs());
	}

	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}




}
