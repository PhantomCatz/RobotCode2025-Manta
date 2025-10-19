package frc.robot.CatzSubsystems.CatzIntakeRollers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Bases.MotorIO;
import frc.robot.Bases.MotorIOReal;
import frc.robot.Utilities.MotorUtil.Gains;

public class RollerConstants {
    // TalonFX motor, double FL, Gains s0g, NeutralModeValue motorMode
    public static final TalonFX RollerMotor = new TalonFX(0);
    public static final double Final_Ratio = 0.0;
    public static final Gains s0g = new Gains(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2); // placeholder ofc
    public static final Gains s1g = new Gains(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2); // placeholder ofcpublic

}