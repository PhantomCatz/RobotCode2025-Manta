package frc.robot.CatzSubsystems.CatzIntakeRollers;



import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Bases.MotorIO;
import frc.robot.CatzSubsystems.CatzArm.ArmConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

public class IntakeRollersIOSim extends MotorIO {
    // CTRE Motor + Sim State
    private final TalonFX m_intakeRollerMotor = new TalonFX(0); // CAN ID 0, adjust as needed

    private final DCMotor m_intakeGearbox = DCMotor.getKrakenX60Foc(1); // TODO: confirm motor type
    private double targetDegreesFinalShaft;

    private PIDController simPIDController = new PIDController(0.0005, 0.0, 0.0);

    private final LinearSystem<N2, N1, N2> plantIntakeMotorSys =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());

    private final DCMotorSim m_IntakeRollersSim = new DCMotorSim(plantIntakeMotorSys, m_intakeGearbox, null);


    public IntakeRollersIOSim() {
        // Configure neutral mode, etc.
        m_intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // PID -> Voltage Command
        //double setVoltage = simPIDController.calculate(inputs.positionDegreesFinalShaft, targetDegreesFinalShaft) * 12.0;

        var m_armSimState = m_intakeRollerMotor.getSimState();
        m_armSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        // Feed input into WPILib sim
        //m_IntakeRollersSim.setInputVoltage(setVoltage);
        m_IntakeRollersSim.update(0.02);


        // Sync CTRE Sim State with WPILib Sim
        m_armSimState.setSupplyVoltage(12.0); // battery voltage
        m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_IntakeRollersSim.getAngularPositionRad()) * ArmConstants.ARM_GEAR_REDUCTION);
        //m_armSimState.setRotorVelocity((Units.radiansPerSecondToRotationsPerMinute(m_IntakeRollersSim.get) * ArmConstants.ARM_GEAR_REDUCTION)/60.0);

        // Fill IO inputs and standered deviations

    }

    @Override
    public void runMotor(double speed) {
        // Set the motor speed in simulation

        //m_intakeRollerMotor.getSimState().setRotorVelocity(speed);
        m_IntakeRollersSim.setInputVoltage(speed * 12.0); // Assuming speed is between -1 and 1
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        // Update the PID controller gains
        simPIDController.setP(kP);
        simPIDController.setI(kI);
        simPIDController.setD(kD);
        // Note: kF is not used in WPILib's PIDController, but can be implemented manually if needed
    }

    @Override
    public void setPosition(double setpointRotations) {
        // Adjust the target position for the intake ramp
        targetDegreesFinalShaft = setpointRotations * 360.0; // Convert rotations to degrees
    }

    //NOTE fill these overrides out

    @Override
    public void runCurrent(double amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCurrent'");
    }

    @Override
    public void setGainsSlot(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot'");
    }

    @Override
    public void setGainsSlot(double kP, double kI, double kD) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot'");
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFF'");
    }

    @Override
    public void runCharacterizationMotor(double input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCharacterizationMotor'");
    }

    @Override
    public void runPercentOutput(double percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runPercentOutput'");
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setBrakeMode'");
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void setCoastOut() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCoastOut'");
    }

    @Override
    public void setNeutralOut() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralOut'");
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentPosition'");
    }

    @Override
    public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicParameters'");
    }

    @Override
    public void setMotionMagicSetpoint(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicSetpoint'");
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocitySetpoint'");
    }

    @Override
    public void setDutyCycleSetpoint(Dimensionless percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDutyCycleSetpoint'");
    }

    @Override
    public void setPositionSetpoint(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionSetpoint'");
    }

    @Override
    public void setVoltageSetpoint(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltageSetpoint'");
    }

    @Override
    public void applySetpoint(Setpoint setpointToApply) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'applySetpoint'");
    }

}
