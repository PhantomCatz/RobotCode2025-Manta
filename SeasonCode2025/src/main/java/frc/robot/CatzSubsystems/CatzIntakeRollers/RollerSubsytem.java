// package frc.robot.CatzSubsystems.CatzIntakeRollers;

// import frc.robot.Bases.TalonFXIOReal;
// import frc.robot.CatzSubsystems.CatzIntakeRollers.RollerSubsytem.intakeRollersStates;
// import frc.robot.Bases.MotorIO;
// import frc.robot.Bases.MotorIO.Setpoint;
// import frc.robot.Bases.MotorSubsystem;

// import static frc.robot.CatzSubsystems.CatzIntakeRollers.RollerConstants.*;


// public class RollerSubsytem extends MotorSubsystem{
//     private static final MotorIO io = new TalonFXIOReal(RollerMotors, Final_Ratio, s0g, s1g, "roller");

//     public static final RollerSubsytem Instance = new RollerSubsytem();

//     //NOTE these subsystem files should only really hold setpoint constants of the mechanism.
//     //e.g. 
//     // public static final Setpoint STOW_FULL = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kFullStowPosition);
// 	// public static final Setpoint STOW_CLEAR = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kStowClearPosition);
// 	// public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kDeployPosition);
// 	// public static final Setpoint EXHAUST = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kExhaustPosition);


//     private RollerSubsytem() {
//         super(io, "CatzIntakeRollers");
//     }


//     //NOTE therefore we shouldn't have these enums and setSpeed methods inside the roller subsystem file. 
//     //states of the mechanisms will be handled in the superstructure class.
//     public enum intakeRollersStates {
//         INTAKE,
//         ANTIJAM,
//         OUTTAKE,
//         STOP
//     }

//     private intakeRollersStates currentState = intakeRollersStates.STOP;
//     private intakeRollersStates previousState = intakeRollersStates.STOP;

//     public static void setSpeed(double speed) {
//         io.runMotor(speed);
//     }
// }
