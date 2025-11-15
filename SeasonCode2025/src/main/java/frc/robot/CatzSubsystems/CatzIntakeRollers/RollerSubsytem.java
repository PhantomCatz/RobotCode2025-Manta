package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.TalonFXIOSim;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorIONull;
import frc.robot.Bases.MotorSubsystem;
import frc.robot.Bases.SparkmaxIOReal;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.CatzConstants;


public class RollerSubsytem extends MotorSubsystem{
    private static final MotorIO io = getIOInstance();

    static MotorIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new SparkmaxIOReal(RollerMotor, Final_Ratio, s0g, IdleMode.kBrake);
            case SIM:
                // System.out.println("Roller Configured for Simulation");
                return new TalonFXIOSim();
            default:
                // System.out.println("Roller Unconfigured");
                return new MotorIONull();
        }
    }

    public static final RollerSubsytem Instance = new RollerSubsytem();


    private RollerSubsytem() {
        super(io, "CatzIntakeRollers");
        io.runMotor(0);
    }

    public enum intakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    private intakeRollersStates currentState = intakeRollersStates.STOP;
    private intakeRollersStates previousState = intakeRollersStates.STOP;

    public static void setSpeed(double speed) {
        io.runMotor(speed);
    }
}
