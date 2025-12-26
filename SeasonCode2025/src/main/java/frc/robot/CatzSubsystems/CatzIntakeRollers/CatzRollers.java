package frc.robot.CatzSubsystems.CatzIntakeRollers;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericMotorIONull;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzAbstractions.io.GenericIOSim;


public class CatzRollers extends GenericMotorSubsystem {
    private static final GenericMotorIO io = getIOInstance();

    static GenericMotorIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new GenericTalonFXIOReal(RollerConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new GenericIOSim();
            default:
                System.out.println("Roller Unconfigured");
                return new GenericMotorIONull();
        }
    }

    public enum IntakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    public static final CatzRollers Instance = new CatzRollers();

    private IntakeRollersStates state = IntakeRollersStates.STOP;

    private CatzRollers() {
        super(io, "CatzIntakeRollers");
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
