package frc.robot.Bases;

import frc.robot.Bases.MotorIO.Setpoint;
import frc.robot.Utilities.DelayedBoolean;
import frc.robot.Utilities.EpsilonEquals;
// import lombok.launch.PatchFixesHider.Util;
// import lombok.launch.PatchFixesHider.Util;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoMotorSubsystem<IO extends MotorIO> extends SubsystemBase {
    protected final IO io;
	protected final String name;

	protected final Angle epsilonThreshold;

	protected boolean isHomingSubsystem;
	private ServoHomingConfig homingConfig;
	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	public ServoMotorSubsystem() {
		super();
		io = null;
		name = null;
		epsilonThreshold = null;
	}

    public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold) {
		super(name);
		this.io = io;
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
	}

    @Override
	public void periodic() {
		io.updateInputs();
		if (isHomingSubsystem) {
			if (mNeedsToHome && setpointNearHome() && nearHomingLocation()) {
				mHoming = true;
				useSoftLimits(false);
				mHomingDelay =
						new DelayedBoolean(Timer.getFPGATimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
			}
			if (mHoming) {
				io.applySetpoint(Setpoint.withVoltageSetpoint(homingConfig.kHomingVoltage));
				if (mHomingDelay.update(
						Timer.getFPGATimestamp(),
						Math.abs(io.getVelocity().baseUnitMagnitude()) < homingConfig.kSetHomedVelocity.baseUnitMagnitude()
								&& DriverStation.isEnabled())) {
					setCurrentPosition(homingConfig.kHomePosition);
					applySetpoint(Setpoint.withMotionMagicSetpoint(homingConfig.kHomePosition));
					useSoftLimits(true);
					mNeedsToHome = false;
				}
			}
		}
	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return io.getCurrentSetpoint().mode.isPositionControl()
				&& EpsilonEquals.epsilonEquals(
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
		return (io.getCurrentSetpoint().mode.isPositionControl()) && nearPosition(io.getPosition());
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return EpsilonEquals.epsilonEquals(
				io.getPositionInch(),
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
