package frc.robot.CatzSubsystems.CatzArm;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.CatzSubsystems.SubystemVisualizer;

public class ArmIOSim implements ArmIO {
    // CTRE Motor + Sim State
    private final TalonFX m_armMotor = new TalonFX(0); // CAN ID 0, adjust as needed

    private final DCMotor m_armGearbox = DCMotor.getKrakenX60Foc(1); // TODO: confirm motor type
    private double targetDegreesFinalShaft;
    private final int ARM_INDEX = 0;

    private PIDController simPIDController = new PIDController(0.0005, 0.0, 0.0);

    private final SingleJointedArmSim m_armSim =
        new SingleJointedArmSim(
            m_armGearbox,
            ArmConstants.ARM_GEAR_REDUCTION,
            ArmConstants.ARM_JKG_SQUARED,
            Units.inchesToMeters(ArmConstants.ARM_LENGTH_INCHES),
            Units.degreesToRadians(ArmConstants.ARM_MIN_DEGREES),
            Units.degreesToRadians(ArmConstants.ARM_MAX_DEGREES),
            true,
            Units.degreesToRadians(ArmConstants.ARM_INITIAL_DEGREES),
            0.01,
            0.0);

    public ArmIOSim() {
        // Configure neutral mode, etc.
        m_armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // PID -> Voltage Command
        double setVoltage = simPIDController.calculate(inputs.positionDegreesFinalShaft, targetDegreesFinalShaft) * 12.0;

        var m_armSimState = m_armMotor.getSimState();
        m_armSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        // Feed input into WPILib sim
        m_armSim.setInputVoltage(setVoltage);
        m_armSim.update(0.02);


        // Sync CTRE Sim State with WPILib Sim
        m_armSimState.setSupplyVoltage(12.0); // battery voltage
        m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_armSim.getAngleRads()) * ArmConstants.ARM_GEAR_REDUCTION);
        m_armSimState.setRotorVelocity((Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()) * ArmConstants.ARM_GEAR_REDUCTION)/60.0);

        // Fill IO inputs
        inputs.positionDegreesFinalShaft = Units.radiansToDegrees(m_armSim.getAngleRads());
        inputs.positionDegCTRE = m_armMotor.getPosition().getValueAsDouble();
        inputs.isArmMotorConnected = true;
        inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec());
        inputs.absoluteEncoderPositionRads = m_armSim.getAngleRads();
        inputs.relativeEncoderPositionRads = m_armSim.getAngleRads();
        inputs.appliedVolts = setVoltage;

        Logger.recordOutput("Arm/Sim target degrees", targetDegreesFinalShaft);

        SubystemVisualizer.setSimPose(ARM_INDEX,
            new Pose3d(
                ArmConstants.ARM_SIM_OFFSET,
                new Rotation3d(0, Units.degreesToRadians(inputs.positionDegreesFinalShaft), 0)
            )
        );
    }

    @Override
    public void runSetpointUp(double setpointDegrees, double feedforwardVolts) {
        System.out.println("setpoint up" + setpointDegrees);
        targetDegreesFinalShaft = setpointDegrees;
    }

    @Override
    public void runSetpointDown(double setpointDegrees, double feedforwardVolts) {
        System.out.println("setpoint down" + setpointDegrees);
        targetDegreesFinalShaft = setpointDegrees;
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        double voltage = percentOutput * 12.0;
        m_armSim.setInputVoltage(voltage);
    }
}
