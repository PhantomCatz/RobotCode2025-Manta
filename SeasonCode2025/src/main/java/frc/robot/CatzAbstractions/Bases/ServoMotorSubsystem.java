package frc.robot.CatzAbstractions.Bases;

import frc.robot.Bases.MotorIOInputsAutoLogged;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.DelayedBoolean;
import frc.robot.Utilities.EpsilonEquals;
import frc.robot.Utilities.Setpoint;
import frc.robot.CatzConstants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoMotorSubsystem extends GenericMotorSubsystem {
    
	protected final GenericMotorIO io;
	protected final String name;
	protected final Angle epsilonThreshold;

	public ServoMotorSubsystem(GenericMotorIO io, String name, Angle epsilonThreshold) {
		super(io, name);
		this.io = io;
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return EpsilonEquals.epsilonEquals(
						getSetpointDoubleInUnits(),
						homingConfig.kHomePosition.in(io.unitType),
						epsilonThreshold.in(io.unitType));
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
		return nearPosition(Angle.ofRelativeUnits(inputs.absoluteEncoderPosition, BaseUnits.AngleUnit.getBaseUnit()));
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return ;
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return EpsilonEquals.epsilonEquals(
				inputs.,
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilonThreshold.in(BaseUnits.AngleUnit));
	}

	public void setCurrentPosition(Angle position) {
		io.setCurrentPosition(position);
	}

	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	public static class ServoHomingConfig {
		public Angle kHomePosition;
		public Voltage kHomingVoltage;
		public Time kHomingTimeout;
		public AngularVelocity kSetHomedVelocity;
	}


}