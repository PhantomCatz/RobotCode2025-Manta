package frc.robot.CatzSubsystems.CatzIntakeRollers;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface RollersIO extends GenericMotorIO<RollersIO.RollersIOInputs> {


    @AutoLog
    public static class RollersIOInputs extends GenericMotorIO.MotorIOInputs {}
}
