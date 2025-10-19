package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.MotorIOReal;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorSubsystem;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


public class RollerSubsytem extends MotorSubsystem{
    private static final MotorIO io = new MotorIOReal(RollerMotor, Final_Ratio, s0g, s1g);

    public static final RollerSubsytem Instance = new RollerSubsytem(io);



    public RollerSubsytem(MotorIO io) {
        super(io, "skibidi"); 
        System.out.println(io == null);
    }

    public enum intakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    private intakeRollersStates currentState = intakeRollersStates.STOP;
    private intakeRollersStates previousState = intakeRollersStates.STOP;


    public static RollerSubsytem getInstance() {
        return Instance;
    }

    public static void setSpeed() {
        io.runMotor(1);
    }
}