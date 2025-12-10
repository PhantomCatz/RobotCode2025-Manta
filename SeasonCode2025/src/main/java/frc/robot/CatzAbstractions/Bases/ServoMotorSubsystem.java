package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.DelayedBoolean;
import frc.robot.Utilities.EpsilonEquals;
import frc.robot.Utilities.Setpoint;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class ServoMotorSubsystem extends GenericMotorSubsystem {

	protected final GenericMotorIO io;
	protected final String name;
	protected final double epsilonThreshold;
	protected ServoHomingConfig homingConfig;
	private Setpoint setpoint;

	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	public ServoMotorSubsystem(GenericMotorIO io, String name, double epsilonThreshold, ServoHomingConfig homingConfig) {
		super(io, name);
		this.io = io;
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
		this.homingConfig = homingConfig;
	}


	@Override
	public final void customGenericPeriodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);

	}

	public void customServoPeriodic() {

	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return EpsilonEquals.epsilonEquals(
						getSetpointDoubleInUnits(),
						homingConfig.kHomePosition,
						epsilonThreshold);
	}

	/**
	 * Determines whether the subsystem is near it's homing position.
	 *
	 * @return True if currently near home position, false if not.
	 */
	public boolean nearHomingLocation() {
		return nearPosition(homingConfig.kHomePosition);
	}

	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not in position control.
	 */
	public boolean nearPositionSetpoint() {
		return nearPosition(inputs.absoluteEncoderPosition);
	}


	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(double mechanismPosition) {
		return EpsilonEquals.epsilonEquals(
				inputs.absoluteEncoderPosition,
				mechanismPosition,
				epsilonThreshold);
	}

	public double getSetpointDoubleInUnits() {
		return setpoint.baseUnits;
	}

	public void setCurrentPosition(double position) {
		io.setCurrentPosition(position);
	}

	public void applySetpoint(Setpoint setpoint) {
		this.setpoint = setpoint;
		io.applySetpoint(setpoint);
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	public static class ServoHomingConfig {
		public double kHomePosition;
		public double kHomingVoltage;
		public double kHomingTimeout;
		public double kSetHomedVelocity;
	}



  public void setBrakeMode(boolean enabled) {
	io.setBrakeMode(enabled);
  }

  public void setFullManual(double manualPower) {
	io.runPercentOutput(manualPower);
  }




}
