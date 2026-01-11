package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.PivotMotorSubsystem;
import frc.robot.Utilities.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;



public class CatzElevator extends PivotMotorSubsystem<ElevatorIO, ElevatorIO.ElevatorIOInputs> {

  private static final ElevatorIO io = getIOInstance();
  private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public static final CatzElevator Instance = new CatzElevator();



  static ElevatorIO getIOInstance() {
    if (io != null) {
      return io;
    } else {
      switch (CatzConstants.hardwareMode) {
          case REAL:
              System.out.println("Elevator Configured for Real");
              return new ElevatorIOTalonFX(ElevatorConstants.getIOConfig());
          case SIM:
              System.out.println("Elevator Configured for Simulation");
              return new ElevatorIOSim(ELEVATOR_GEAR_RATIO, slot0_gains);
          default:
              System.out.println("Elevator Unconfigured; defaulting to Simulation");
              return new ElevatorIOSim(ELEVATOR_GEAR_RATIO, slot0_gains);
      }
    }
  }


  private CatzElevator() {
    super(io, inputs, "CatzElevator", 0.5);
  }

  @Override
  public void periodic() {
    super.periodic();

    if(inputs.forwardLimitSwitch) {
      io.setCurrentPosition(0.0);
    }

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

    //---------------------------------------------------------------------------------------------------------------------------
    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------
    Logger.recordOutput("Elevator/targetPosition", 0.0);
  }

  public boolean getBottomLimitSwitch() {
    return inputs.forwardLimitSwitch;
  }


}
