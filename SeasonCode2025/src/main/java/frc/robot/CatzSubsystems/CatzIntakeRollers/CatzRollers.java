package frc.robot.CatzSubsystems.CatzIntakeRollers;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;


public class CatzRollers extends GenericMotorSubsystem {
    private static final RollersIO io = getIOInstance();

    static RollersIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new RollerIOTalonFX(RollerConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new RollersIOSimulator();
                default:
                System.out.println("Roller Unconfigured");
                return new RollersIOSimulator();
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
