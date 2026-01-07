package frc.robot.CatzSubsystems.CatzElevator;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.DigitalInOut;
import frc.robot.CatzAbstractions.Bases.PivotMotorSubsystem;
import frc.robot.CatzAbstractions.io.DigitalInOutIOBeambreak;
import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericMotorIONull;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.Utilities.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;


public class CatzElevator extends PivotMotorSubsystem {

  private static final GenericMotorIO io = getIOInstance();

  public static final CatzElevator Instance = new CatzElevator();


  private DigitalInOut bottomLimitSwitch = new DigitalInOut(
      new DigitalInOutIOBeambreak(3, false),
      Time.ofBaseUnits(0.02, Seconds), // TODO find better debounce time
      false,
      "Elevator/BottomLimitSwitch"
  );

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
              return new GenericIOSim(ELEVATOR_GEAR_RATIO, slot0_gains);
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
    bottomLimitSwitch.periodic();
    super.periodic();

    if(bottomLimitSwitch.get()) {
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
    return bottomLimitSwitch.get();
  }


}
