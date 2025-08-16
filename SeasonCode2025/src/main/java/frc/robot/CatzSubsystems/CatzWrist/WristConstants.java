package frc.robot.CatzSubsystems.CatzWrist;

import frc.robot.Utilities.MotorUtil.Gains;


import frc.robot.CatzConstants;

public class WristConstants {
    public static final boolean isWristDisabled = false;

    public static final double WRIST_MOTOR_GEAR_REDUCTION = 3.0; //TODO 
    public static final int WRIST_MOTOR_ID = 50;
    public static final double WRIST_RETRACT = 0.0;
    public static final double WRIST_CATCH = 0.0;

    // Positive is winching in negative is winching out

    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            //(100.0, 0.0, 0.0, 0.25, 0.12, 0.01, 0.0);
            case SN2 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //TBD FOR GODSAKE
            case SN1 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
}
