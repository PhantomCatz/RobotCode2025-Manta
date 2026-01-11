package frc.robot.CatzSubsystems.CatzIntakeRollers;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class RollerIOTalonFX extends GenericTalonFXIOReal<RollersIO.RollersIOInputs> implements RollersIO {

    public RollerIOTalonFX(MotorIOTalonFXConfig config) {
        super(config);
    }

}
