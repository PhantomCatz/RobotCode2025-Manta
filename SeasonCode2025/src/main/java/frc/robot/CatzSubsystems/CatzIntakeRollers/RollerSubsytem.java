package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.MotorIOReal;
import frc.robot.Bases.MotorSubsystem;

public class RollerSubsytem extends MotorSubsystem{
    public static final RollerSubsytem Instance = new RollerSubsytem();

    private static final MotorIOReal RollerIO = RollerConstants.getRollerIO();

    public RollerSubsytem() {
        super(RollerIO, "skibidi");
    }

    public enum intakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    private intakeRollersStates currentState = intakeRollersStates.STOP;
    private intakeRollersStates previousState = intakeRollersStates.STOP;

    @Override
    public void periodic() {
        RollerIO.updateInputs(RollerIO.getMotorIOInputs());
    }

    public static MotorIOReal getRollerIO() {
        return RollerIO;
    }

    public static RollerSubsytem getInstance() {
        return Instance;
    }

    public static void setSpeed() {
        RollerIO.runMotor(1);
    }
}