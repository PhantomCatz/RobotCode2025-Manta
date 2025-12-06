package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorIO.Setpoint;
import frc.robot.Bases.MotorIOInputsAutoLogged;
import frc.robot.Bases.ServoMotorSubsystem;
import frc.robot.Bases.TalonFXIONull;
import frc.robot.Bases.TalonFXIOReal;
import frc.robot.Bases.TalonFXIOSim;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CatzElevatorNew extends ServoMotorSubsystem{
    public static final CatzElevatorNew Instance = new CatzElevatorNew();

    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    public static final Setpoint L4_SCORE =
			Setpoint.withPositionSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL4ScoringHeight));
	public static final Setpoint L3_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL3ScoringHeight));
	public static final Setpoint L2_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2ScoringHeight));
	public static final Setpoint L1_SCORE =

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



}
