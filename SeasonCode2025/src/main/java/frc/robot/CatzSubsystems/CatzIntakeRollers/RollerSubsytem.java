package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.TalonFXIOReal;
import frc.robot.Bases.TalonFXIOSim;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorSubsystem;
import frc.robot.Bases.TalonFXIONull;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;

import frc.robot.Robot;


public class RollerSubsytem extends MotorSubsystem{
    private static final MotorIO io = getIOInstance();

    static MotorIO getIOInstance() {
        if(Robot.isReal()) {
            return new TalonFXIOReal(RollerMotor, Final_Ratio, s0g, s1g);
        } else if(Robot.isSimulation()) {
            return new TalonFXIOSim();
        } else {
            return new TalonFXIONull();
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
