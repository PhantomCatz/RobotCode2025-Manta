package frc.robot.CatzSubsystems.CatzWrist;

import static frc.robot.CatzSubsystems.CatzWrist.WristConstants.*;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzArm.ArmConstants;

/** Add your docs here. */
public class WristIOSim implements WristIO {

  private final DCMotor m_wristMotor = DCMotor.getKrakenX60Foc(1);
  private double targetDegreesFinalShaft;

  private final int WRIST_MOTOR_INDEX = 2;

  private PIDController simPidController = new PIDController(0.001, 0.0, 0.0);
  private final SingleJointedArmSim m_wristMotorSim =
  new SingleJointedArmSim(
      m_wristMotor,
      WristConstants.WRIST_MOTOR_GEAR_REDUCTION,
      0.025, // Used swerve drive jkg squared
      Units.inchesToMeters(16.0),
      -Math.PI,
      Math.PI, // Bounds seemed too small
      true,
      Units.degreesToRadians(WristConstants.WRIST_RETRACT), // initial position was intially out of bounds with the math.pi/2 < 1.58
      0.01,
      0.0);

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.positionDegrees = Units.radiansToDegrees(m_wristMotorSim.getAngleRads());
    inputs.velocityRpm = Units.radiansPerSecondToRotationsPerMinute(m_wristMotorSim.getVelocityRadPerSec());
  
    double setVoltage = simPidController.calculate(inputs.positionDegrees, targetDegreesFinalShaft) * 12.0;
    m_wristMotorSim.update(0.02);
    m_wristMotorSim.setInputVoltage(setVoltage);

    Robot.setSimPose(WRIST_MOTOR_INDEX, new Pose3d(ArmConstants.ARM_SIM_OFFSET, new Rotation3d(0, Units.degreesToRadians(inputs.positionDegrees), 0)));
  }

  @Override
  public void setPosition(double targetDegrees) // Set the motor position in mechanism rotations
  {
    targetDegreesFinalShaft = targetDegrees;
  }

  // @Override
  // public void setPower(double power) {
  //   // System.out.println("wrist set power: " + power);
  //   wristMotor.set(power);
  // }

  // @Override
  // public void setPID(double kP, double kI, double kD) {
  //   config.Slot0.kP = kP;
  //   config.Slot0.kI = kI;
  //   config.Slot0.kD = kD;
  //   System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
  //   wristMotor.getConfigurator().apply(config);
  // }

  // @Override
  // public void runCharacterizationMotor(double input) {
  //   wristMotor.setControl(voltageControl.withOutput(input));
  // }

  // @Override
  // public void setFF(double kS, double kV, double kA) {
  //   config.Slot0.kS = kS;
  //   config.Slot0.kV = kV;
  //   config.Slot0.kA = kA;
  //   System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
  //   wristMotor.getConfigurator().apply(config);
  // }

}
