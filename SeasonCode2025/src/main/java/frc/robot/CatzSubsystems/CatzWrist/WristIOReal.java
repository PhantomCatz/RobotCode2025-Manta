package frc.robot.CatzSubsystems.CatzWrist;

import static frc.robot.CatzSubsystems.CatzWrist.WristConstants.*;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class WristIOReal implements WristIO {
  TalonFX wristMotor = new TalonFX(WRIST_MOTOR_ID);

  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristSupplyCurrent;
  private final StatusSignal<Current> wristTorqueCurrent;
  private final StatusSignal<Temperature> wristTempCelsius;
  private final StatusSignal<Double> wristDutyCycle;

  public WristIOReal() {
    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();
    wristAppliedVolts = wristMotor.getMotorVoltage();
    wristSupplyCurrent = wristMotor.getSupplyCurrent();
    wristTorqueCurrent = wristMotor.getTorqueCurrent();
    wristTempCelsius = wristMotor.getDeviceTemp();
    wristDutyCycle = wristMotor.getDutyCycle();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristSupplyCurrent,
        wristTorqueCurrent,
        wristTempCelsius);

    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();

    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    config.MotionMagic.MotionMagicCruiseVelocity = 100;
    config.MotionMagic.MotionMagicAcceleration = 600;
    config.MotionMagic.MotionMagicJerk = 2000;

    wristMotor.getConfigurator().apply(config, 1.0);

    wristMotor.setPosition(0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.isPositionIOMotorConnected =
        BaseStatusSignal.refreshAll(
                wristPosition,
                wristVelocity,
                wristAppliedVolts,
                wristSupplyCurrent,
                wristTorqueCurrent,
                wristTempCelsius)
            .isOK();
    inputs.positionDegrees = (wristPosition.getValueAsDouble() / WRIST_MOTOR_GEAR_REDUCTION) * 360.0;
    inputs.velocityRpm = wristVelocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = wristTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = wristTempCelsius.getValueAsDouble();
    inputs.commandedOutput = wristDutyCycle.getValueAsDouble();
  }

  @Override
  public void setPosition(double pos) // Set the motor position in mechanism rotations
  {
    wristMotor.setControl(positionControl.withPosition(pos));
    // System.out.println(pos);
  }

  @Override
  public void setPower(double power) {
    // System.out.println("wrist set power: " + power);
    wristMotor.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    wristMotor.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationMotor(double input) {
    wristMotor.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
    wristMotor.getConfigurator().apply(config);
  }

}
