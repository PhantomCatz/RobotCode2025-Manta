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

	//NOTE: try adding these code into the periodic (they are from 1678's code) Idk the exact reason for using this but it probably allows for accurate homing
	/*

	 * @Override
	public void periodic() {
		super.periodic();
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
						Math.abs(getVelocity().baseUnitMagnitude()) < homingConfig.kSetHomedVelocity.baseUnitMagnitude()
								&& DriverStation.isEnabled())) {
					setCurrentPosition(homingConfig.kHomePosition);
					applySetpoint(Setpoint.withMotionMagicSetpoint(homingConfig.kHomePosition));
					useSoftLimits(true);
					mNeedsToHome = false;
				}
			}
		}
	}

	 */

	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}




}
