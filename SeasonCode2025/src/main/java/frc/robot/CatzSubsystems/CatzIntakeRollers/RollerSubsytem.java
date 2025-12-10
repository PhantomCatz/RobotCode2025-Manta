package frc.robot.CatzSubsystems.CatzIntakeRollers;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericMotorIONull;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzAbstractions.io.GenericIOSim;


public class RollerSubsytem extends GenericMotorSubsystem {
    private static final GenericMotorIO io = getIOInstance();

    static GenericMotorIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new GenericTalonFXIOReal(
                    0.25,
                    s0g,
                    NeutralModeValue.Brake,
                    new TalonFX(45),
                    new TalonFX(46)
                );
            case SIM:
                // System.out.println("Roller Configured for Simulation");
                return new GenericIOSim();
            default:
                // System.out.println("Roller Unconfigured");
                return new GenericMotorIONull();
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
