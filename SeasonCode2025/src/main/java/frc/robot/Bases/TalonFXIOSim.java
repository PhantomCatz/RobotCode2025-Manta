package frc.robot.Bases;



import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CatzSubsystems.CatzArm.ArmConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

public class TalonFXIOSim implements MotorIO {
    // CTRE Motor + Sim State
    private final TalonFX LeaderTalon = new TalonFX(0); // CAN ID 0, adjust as needed

    private final DCMotor GearBox = DCMotor.getKrakenX60Foc(1); // TODO: confirm motor type 67
    private double targetDegreesFinalShaft;

    private PIDController simPIDController = new PIDController(0.0005, 0.0, 0.0);

    private final LinearSystem<N2, N1, N2> plantIntakeMotorSys =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());

    private final DCMotorSim TalonFXSim = new DCMotorSim(plantIntakeMotorSys, GearBox, null);


    public TalonFXIOSim() {
        // Configure neutral mode, etc.
        LeaderTalon.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // PID -> Voltage Command
        //double setVoltage = simPIDController.calculate(inputs.positionDegreesFinalShaft, targetDegreesFinalShaft) * 12.0;

        var TalonFXSimState = LeaderTalon.getSimState();
        TalonFXSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        // Feed input into WPILib sim
        //m_IntakeRollersSim.setInputVoltage(setVoltage);
        TalonFXSim.update(0.02);


        // Sync CTRE Sim State with WPILib Sim
        TalonFXSimState.setSupplyVoltage(12.0); // battery voltage
        TalonFXSimState.setRawRotorPosition(Units.radiansToRotations(TalonFXSim.getAngularPositionRad()) * ArmConstants.ARM_GEAR_REDUCTION);
        //m_armSimState.setRotorVelocity((Units.radiansPerSecondToRotationsPerMinute(m_IntakeRollersSim.get) * ArmConstants.ARM_GEAR_REDUCTION)/60.0);

        // Fill IO inputs and standered deviations

    }

    @Override
    public void runMotor(double speed) {
        // Set the motor speed in simulation

        //LeaderTalon.getSimState().setRotorVelocity(speed);
        TalonFXSim.setInputVoltage(speed * 12.0); // Assuming speed is between -1 and 1
    }

    @Override
    public void setGainsSlot(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
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

}
