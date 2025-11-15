package frc.robot.Bases;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public abstract class MotorIO {

	/*
	 * NOTE probably a better idea to turn this into an abstract class to properly log
	 * follower motors and
	 * because there is no reason for this to be an interface. abstract classes allow for more flexibility
	 */

	//NOTE a single MotorIO will represent an entire group of motors that work together. lowkey why don't we just have one array that holds all of the motors instead of spliiting it into two?
	protected final MotorIOInputs motorInputs;
	protected final MotorIOInputs[] followerInputs; //NOTE always use arrays instead of arraylist to reduce room for error.


	public MotorIO(int numFollowers){
		motorInputs = new MotorIOInputs();
		followerInputs = new MotorIOInputs[numFollowers];
	}

	@AutoLog
	public static class MotorIOInputs {

		public boolean isMotorConnected = false;

		public double motorRotations = 0.0;
		public double absoluteEncoderPositionRads = 0.0;
		public double relativeEncoderPositionRads = 0.0;
		public double velocityInchPerSec = 0.0;
		public double acceleration = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double torqueCurrentAmps = 0.0;
		public double appliedVoltage = 0.0;
		public double tempCelcius = 0.0;

	}

	public abstract void updateInputs(MotorIOInputs inputs);

	public abstract void runMotor(double speed);

	public abstract void runCurrent(double amps);

	public abstract void setGainsSlot(double kP, double kI, double kD, double kS, double kV, double kA, double kG);

	public abstract void setGainsSlot(double kP, double kI, double kD);

	public abstract void setFF(double kS, double kV, double kA);

	public abstract void runCharacterizationMotor(double input);

	public abstract void runPercentOutput(double percent);

	public abstract void setPosition(double pos);

	public abstract void setBrakeMode(boolean enabled);

	public abstract void setNeutralMode(NeutralModeValue mode);

	public abstract void setIdleMode(IdleMode mode);

	public abstract void stop();

	public MotorIOInputs getMotorIOInputs() {
		return new MotorIO.MotorIOInputs();
	}

	public abstract void setCoastOut();

	public abstract void setNeutralOut();

	public abstract void setCurrentPosition(Angle mechanismPosition);

	public abstract void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk);

	public abstract void setMotionMagicSetpoint(Angle mechanismPosition);

	public abstract void setVelocitySetpoint(AngularVelocity mechanismVelocity);

	public abstract void setDutyCycleSetpoint(Dimensionless percent);

	public abstract void setPositionSetpoint(Angle mechanismPosition);

	public abstract void setVoltageSetpoint(Voltage voltage);

	public abstract void applySetpoint(Setpoint setpointToApply);

	public double getVelocityInch() {
		return 0.0;
	}

	public double getPositionInch() {
		return 0.0;
	}

	public double getSupplyCurrent() {
		return 0.0;
	}

	//NOTE write the rest of get functions

	public enum Mode {
		IDLE,
		VOLTAGE,
		MOTIONMAGIC,
		VELOCITY,
		DUTY_CYCLE,
		POSITIONPID;

		/**
		 * Gets whether the control mode is based on position. Motion Magic and Position
		 * PID control count as position.
		 *
		 * @return True if in position control, false if not.
		 */
		public boolean isPositionControl() {
			return switch (this) {
				case MOTIONMAGIC, POSITIONPID -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on velocity.
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isVelocityControl() {
			return switch (this) {
				case VELOCITY -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is neutral. Only Idle counts as neutral
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isNeutralControl() {
			return switch (this) {
				case IDLE -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on voltage. Voltage and Duty Cycle control count as voltage.
		 *
		 * @return True if in voltage control, false if not.
		 */
		public boolean isVoltageControl() {
			return switch (this) {
				case VOLTAGE, DUTY_CYCLE -> true;
				 default -> false;
			};
		}
	}

	public static class Setpoint {
		private final UnaryOperator<MotorIO> applier;
		public final Mode mode;
		public final double baseUnits;

		/**
		 * Creates a setpoint with a given applier, control mode, and base units
		 * equivalent.
		 *
		 * @param applier   What to apply to ServoMotorIO when the setpoint is set.
		 * @param mode      Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		private Setpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			this.applier = applier;
			this.mode = mode;
			this.baseUnits = baseUnits;
		}

		/**
		 * Creates a setpoint with a completely custom applier, control mode, and base
		 * units.
		 *
		 * @param applier   What to apply to ServoMotorIO when the setpoint is set.
		 * @param mode      Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		public static Setpoint withCustomSetpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			return new Setpoint(applier, mode, baseUnits);
		}

		/**
		 * Creates a setpoint to use motion magic control to go to a position.
		 *
		 * @param motionMagicSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpoint(Angle motionMagicSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setMotionMagicSetpoint(motionMagicSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use PID control to go to a position.
		 *
		 * @param positionSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withPositionSetpoint(Angle positionSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setPositionSetpoint(positionSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpoint(AngularVelocity velocitySetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVelocitySetpoint(velocitySetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a voltage.
		 *
		 * @param voltage Voltage to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVoltageSetpoint(Voltage voltage) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVoltageSetpoint(voltage);
				return io;
			};
			return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a percent of maximum voltage.
		 *
		 * @param percent Percent to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setDutyCycleSetpoint(percent);
				return io;
			};
			return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to idle.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withNeutralSetpoint() {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setNeutralOut();
				return io;
			};
			return new Setpoint(applier, Mode.IDLE, 0.0);
		}

		/**
		 * Creates a setpoint to coast.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withCoastSetpoint() {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setCoastOut();
				return io;
			};
			return new Setpoint(applier, Mode.IDLE, 0.0);
		}

		public void apply(MotorIO io) {
			applier.apply(io);
		}
	}

}
