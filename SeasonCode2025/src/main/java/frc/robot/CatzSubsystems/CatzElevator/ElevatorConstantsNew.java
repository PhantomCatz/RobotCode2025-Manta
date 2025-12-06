package frc.robot.CatzSubsystems.CatzElevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class ElevatorConstantsNew {
    public static final int LEFT_LEADER_ID  = 31;
    public static final int RIGHT_FOLLOWER_ID = 30;

    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            //case SN2 -> new Gains(8.0, 0.0, 0.0, 0.175, 0.230, 0.013, 0.4);
            case SN2 -> new Gains(10.0, 0.0, 0.0, 0.065, 0.379, 0.009, 0.0);//            case SN2 -> new Gains(4.0, 0.0, 0.0, 0.175, 0.425, 0.022, 0.0);

            case SN1 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.3, 0.013, 0.4); //

            case SN_TEST, SN1_2024 -> new Gains(4.0, 0.0, 0.0, 0.065, 0.379, 0.015, 0.0);
        };

    public static final Gains slot1_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.130, 0.009, 0.4);
                               // v 75.0 vv 0.1 v
            case SN1 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.13, 0.013, 0.4); //TODO
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final Angle L1_HEIGHT = new Angle();
}
