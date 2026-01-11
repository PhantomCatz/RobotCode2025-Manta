package frc.robot.CatzSubsystems.CatzElevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ElevatorIO extends GenericMotorIO<ElevatorIO.ElevatorIOInputs> {

    @AutoLog
    public static class ElevatorIOInputs extends GenericMotorIO.MotorIOInputs {
        public boolean forwardLimitSwitch = false;
        public boolean reverseLimitSwitch = false;
    }

    // You can also add unique methods here
    public default void resetAtLimit() {}
}
