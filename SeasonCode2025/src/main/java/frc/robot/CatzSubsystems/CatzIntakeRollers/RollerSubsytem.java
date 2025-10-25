package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.TalonFXIOReal;
import frc.robot.CatzSubsystems.CatzIntakeRollers.RollerSubsytem.intakeRollersStates;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorSubsystem;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


public class RollerSubsytem extends MotorSubsystem{
    private static final MotorIO io = new TalonFXIOReal(RollerMotor, Final_Ratio, s0g, s1g);

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
