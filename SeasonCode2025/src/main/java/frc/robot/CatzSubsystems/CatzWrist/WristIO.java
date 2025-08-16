package frc.robot.CatzSubsystems.CatzWrist;


import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean isPositionIOMotorConnected = true;

    public double positionDegrees = 0.0;
    public double sparkPosMechs;
    public double velocityRpm = 0.0;
    public double velocityRads = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double commandedOutput = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(double pos) {}

  public default void runSetpointTicks(double setpointTicks) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  //   public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPower(double joystickPower) {}

}
