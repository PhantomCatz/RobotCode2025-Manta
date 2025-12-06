package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstantsNew.*;

import frc.robot.CatzConstants;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.ServoMotorSubsystem;
import frc.robot.Bases.TalonFXIONull;
import frc.robot.Bases.TalonFXIOReal;
import frc.robot.Bases.TalonFXIOSim;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CatzElevatorNew extends ServoMotorSubsystem<TalonFXIOReal>{
    public static final CatzElevatorNew Instance = new CatzElevatorNew();

    private static MotorIO io;
   
    private CatzElevatorNew() {
        if(isElevatorDisabled) {
          io = new TalonFXIONull();
          System.out.println("Elevator Unconfigured");
        } else {
          switch (CatzConstants.hardwareMode) {
            case REAL:
              io = new TalonFXIOReal(FINAL_RATIO, slot0_gains, slot1_gains, NeutralModeValue.Brake, true, "elevator", elevatorMotors);
              System.out.println("Elevator Configured for Real");
            break;
            case REPLAY:
              io = new TalonFXIOReal(FINAL_RATIO, slot0_gains, slot1_gains, NeutralModeValue.Brake, true, "elevator", elevatorMotors) {};
              System.out.println("Elevator Configured for Replayed simulation");
            break;
            case SIM:
              io = new TalonFXIOSim();
              System.out.println("Elevator Configured for Simulation");
            break;
            default:
              io = new TalonFXIONull();
              System.out.println("Elevator Unconfigured");
            break;
          }
        }
        // SmartDashboard.putData("Mech2d", m_mech2d);
      }

      public static void Stow() {
        io.applySetpoint(ElevatorConstantsNew.STOW);
      }

      public static void L1() {
        io.applySetpoint(ElevatorConstantsNew.L1_SCORE);
      }

      public static void L2() {
        io.applySetpoint(ElevatorConstantsNew.L2_SCORE);
      }

      public static void L3() {
        io.applySetpoint(ElevatorConstantsNew.L3_SCORE);
      }

      public static void L4() {
        io.applySetpoint(ElevatorConstantsNew.L4_SCORE);
      }

}
