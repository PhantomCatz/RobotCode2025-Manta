package frc.robot.CatzSubsystems.CatzWrist;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface WristIO extends GenericMotorIO<WristIO.WristIOInputs> {

    @AutoLog
    public static class WristIOInputs extends GenericMotorIO.MotorIOInputs {}
}
