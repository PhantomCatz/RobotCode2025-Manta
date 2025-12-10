package frc.robot.CatzAbstractions.io;


import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Utilities.Setpoint;

public interface GenericMotorIO {

  @AutoLog
  public static class MotorIOInputs {

    public boolean isLeaderConnected = false;
    public boolean isFollowerConnected = false;

    public double relativeEncoderPosition = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double velocityRPS = 0.0;
    public double accelerationRPS = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

  }

  public default void updateInputs(MotorIOInputs inputs) {}

  public default void runMotor(double Speed) {}

  public default void runMotorBck(double Speed) {}

  public default void runCurrent(double amps) {}

  public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setGainsSlot0(double kP, double kI, double kD) {}

  public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setGainsSlot1(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runPercentOutput(double percent) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setNeutralMode(NeutralModeValue mode) {}

  public default void setIdleMode(IdleMode mode) {}

  public default void stop() {}

  public default void setNeutralOut() {}

  public default void setCurrentPosition(double mechanismPosition) {}

  public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

  public default void setMotionMagicSetpoint(double mechanismPosition) {}

  public default void setVelocitySetpoint(double mechanismVelocity) {}

  public default void setDutyCycleSetpoint(double percent) {}

  public default void setPositionSetpoint(double mechanismPosition) {}

  public default void setVoltageSetpoint(double voltage) {}

  public default void applySetpoint(Setpoint setpointToApply) {}

  public default void useSoftLimits(boolean enable) {}

  public default int getNumMotors() {
    return 1;
  }

}
