package frc.robot.CatzSubsystems.CatzIntakeRollers;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.SubystemVisualizer;
import frc.robot.CatzSubsystems.CatzArm.ArmConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

public class IntakeRollersIOSim implements IntakeRollersIO {
    // CTRE Motor + Sim State
    private final TalonFX m_intakeRollerMotor = new TalonFX(0); // CAN ID 0, adjust as needed

    private double targetDegreesFinalShaft;

    private PIDController simPIDController = new PIDController(0.0005, 0.0, 0.0);

    private final DCMotor m_intakeGearbox = DCMotor.getKrakenX60Foc(1); // TODO: confirm motor type

    private final LinearSystem<N2, N1, N2> plantIntakeMotorSys =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());

    private final DCMotorSim m_IntakeRollersSim = new DCMotorSim(plantIntakeMotorSys, m_intakeGearbox, 0.0, 0.0);


    public IntakeRollersIOSim() {
        // Configure neutral mode, etc.
        m_intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        // PID -> Voltage Command
        double setVoltage = simPIDController.calculate(inputs.positionDegreesFinalShaft, targetDegreesFinalShaft) * 12.0;

        var m_armSimState = m_intakeRollerMotor.getSimState();
        m_armSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        // Feed input into WPILib sim
        m_IntakeRollersSim.update(0.02);

        inputs.velocityRpmLeft = Units.radiansPerSecondToRotationsPerMinute(m_IntakeRollersSim.getAngularVelocityRadPerSec());

        // Sync CTRE Sim State with WPILib Sim
        m_armSimState.setSupplyVoltage(12.0); // battery voltage
        m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_IntakeRollersSim.getAngularPositionRad()) * ArmConstants.ARM_GEAR_REDUCTION);
        m_armSimState.setRotorVelocity((Units.radiansPerSecondToRotationsPerMinute(m_IntakeRollersSim.get) * ArmConstants.ARM_GEAR_REDUCTION)/60.0);

        // Fill IO inputs

    }

    @Override
    public void runIntakeRampMotor(double speed) {
        // Set the motor speed in simulation
        m_IntakeRollersSim.setInputVoltage(speed * 12.0); // Assuming speed is between -1 and 1
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kF) {
        // Update the PID controller gains
        simPIDController.setP(kP);
        simPIDController.setI(kI);
        simPIDController.setD(kD);
        // Note: kF is not used in WPILib's PIDController, but can be implemented manually if needed
    }

    @Override
    public void adjustIntakeRamp(double setpointRotations) {
        // Adjust the target position for the intake ramp
        targetDegreesFinalShaft = setpointRotations * 360.0; // Convert rotations to degrees
    }

}
