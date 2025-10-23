package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.Bases.TalonFXIOReal;
import frc.robot.CatzSubsystems.CatzIntakeRollers.RollerSubsytem.intakeRollersStates;
import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorSubsystem;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


public class RollerSubsytem extends MotorSubsystem{
    private static final MotorIO io = new TalonFXIOReal(RollerMotor, Final_Ratio, s0g, s1g);

    public static final RollerSubsytem Instance = new RollerSubsytem(io);



    public RollerSubsytem(MotorIO io) {
        super(io, "CatzIntakeRollers"); 
    }

    // @Override
    // public void periodic() {
    //     io.updateInputs(inputs);
    // }

    public enum intakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    private intakeRollersStates currentState = intakeRollersStates.STOP;
    private intakeRollersStates previousState = intakeRollersStates.STOP;

    public static void setSpeed(int speed) {
        io.runMotor(speed);
        //System.out.println("wozerwsdadsadwdsad it set speed");
    }
}