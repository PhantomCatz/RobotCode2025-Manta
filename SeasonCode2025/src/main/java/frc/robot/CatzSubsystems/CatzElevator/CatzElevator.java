package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericMotorIONull;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.Utilities.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;


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
              return new GenericTalonFXIOReal(ElevatorConstants.getIOConfig());
          case SIM:
              System.out.println("Elevator Configured for Simulation");
              return new GenericIOSim();
          default:
              System.out.println("Elevator Unconfigured");
              return new GenericMotorIONull();
      }
    }
  }


  private CatzElevator() {
    super(io, "CatzElevator", 0.5, 4.0);
  }

  @Override
  public void periodic() {
    super.periodic();
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


}
