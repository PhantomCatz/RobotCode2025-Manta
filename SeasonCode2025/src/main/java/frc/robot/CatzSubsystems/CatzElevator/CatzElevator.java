package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;
import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.s0g;

import java.util.function.DoubleSupplier;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericMotorIONull;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CatzElevator extends ServoMotorSubsystem {

  public static final CatzElevator Instance = new CatzElevator();

  private static final GenericMotorIO io = getIOInstance();

  static GenericMotorIO getIOInstance() {
    if (io != null) {
      return io;
    } else {
      switch (CatzConstants.hardwareMode) {
          case REAL:
              System.out.println("Elevator Configured for Real");
              return new GenericTalonFXIOReal(
                  0.25,
                  s0g,
                  NeutralModeValue.Brake,
                  new TalonFX(45),
                  new TalonFX(46)
              );
          case SIM:
              // System.out.println("Roller Configured for Simulation");
              return new GenericIOSim();
          default:
              // System.out.println("Roller Unconfigured");
              return new GenericMotorIONull();
      }
    }
  }

  private double elevatorSpeed = 0.0;
  private double elevatorFeedForward = 0.0;
  private int settlingCounter = 0;
  private boolean breakModeEnabled = true;

  private ElevatorPosition targetPosition = ElevatorPosition.PosStow;
  private ElevatorPosition prevTargetPositon = ElevatorPosition.PosNull;
  private ElevatorPosition previousLoggedPosition = ElevatorPosition.PosNull;

  private boolean isElevatorInPos = false;

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      //TO CHANGE HEIGHT GO TO ElevatorConstants.java
      PosLimitSwitchStow(() -> 0.0),
      PosStow(() -> STOW_HEIGHT),
      PosCoastStow(() -> COAST_STOW_HEIGHT),
      PosL1(() -> L1_HEIGHT),
      PosL2(() -> L2_HEIGHT),
      PosL3(() -> L3_HEIGHT),
      PosL4(() -> L4_HEIGHT),
      PosL4Adj(() -> L4_CORAL_ADJ),
      AlgaeBotTransition(() -> ALGAE_BOT),
      PosBotBot(() -> BOT_BOT_ALGAE),
      PosBotTop(() -> BOT_TOP_ALGAE),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0)),
      PosNull(() -> -1.0);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionInch() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  private CatzElevator() {
    super(io, "CatzElevator", 0.5, getServoConfig());
  }

  @Override
  public void customServoPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/Elevator", inputs);

    //--------------------------------------------------------------------------------------------------------
    // Update controllers when user specifies
    //--------------------------------------------------------------------------------------------------------
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot0(slot0_kP.get(),
                               slot0_kI.get(),
                               slot0_kD.get(),
                               slot0_kS.get(),
                               slot0_kV.get(),
                               slot0_kA.get(),
                               slot0_kG.get()
        ),
        slot0_kP,
        slot0_kI,
        slot0_kD,
        slot0_kS,
        slot0_kV,
        slot0_kA,
        slot0_kG
    );

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot1(slot1_kP.get(),
                               slot1_kI.get(),
                               slot1_kD.get(),
                               slot1_kS.get(),
                               slot1_kV.get(),
                               slot1_kA.get(),
                               slot1_kG.get()
        ),
        slot1_kP,
        slot1_kI,
        slot1_kD,
        slot1_kS,
        slot1_kV,
        slot1_kA,
        slot1_kG
    );

    LoggedTunableNumber.ifChanged(hashCode(),
        ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()),
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);

    if(previousLoggedPosition != targetPosition) {
      prevTargetPositon = targetPosition;
    }

    //---------------------------------------------------------------------------------------------------------------------------

    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(DriverStation.isDisabled()) {
      // Disabled
      io.stop();
      targetPosition = ElevatorPosition.PosNull;
    } else if(targetPosition != ElevatorPosition.PosNull &&
              targetPosition != ElevatorPosition.PosManual){
      // if(targetPosition == ElevatorPosition.PosCoastStow) {
      //   //hopefully a slow descent from algae intake positions to stow w an algae; regular PID effected desecent is too extreme for algae pivot arm
      //   io.stop();
      // Setpoint PID
      if(targetPosition == ElevatorPosition.PosStow) {
        // Safety Stow
        if(this.getPosition() < 2.0) {
          io.stop();
        } else {
          io.applySetpoint(null);
          // System.out.println("setpoint down");
        }
      } else {
        //Setpoint PID
        io.applySetpoint(null);
        // System.out.println("setpoint up");

      }
    } else if (targetPosition == ElevatorPosition.PosManual) {
      io.runMotor(elevatorSpeed);

      Logger.recordOutput("Elevator/Manual Speed", elevatorSpeed*10);
      Logger.recordOutput("Elevator/Manual RPS", inputs.velocityRPS);
    } else {
      // Nothing happening
      io.stop();
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------


   // Logger.recordOutput("Elevator/CurrentPosition", getElevatorPositionInch());
    Logger.recordOutput("Elevator/prevtargetPosition", prevTargetPositon.getTargetPositionInch());
    Logger.recordOutput("Elevator/logged prev targetPosition", previousLoggedPosition.getTargetPositionInch());
    //Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());
    Logger.recordOutput("Elevator/targetPosition", targetPosition.getTargetPositionInch());


    // Target Postioin Logging
    previousLoggedPosition = targetPosition;
  }


}
