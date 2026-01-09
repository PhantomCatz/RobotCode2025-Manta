package frc.robot.CatzSubsystems.CatzWrist;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class WristIOTalonFX extends GenericTalonFXIOReal implements WristIO {

    public WristIOTalonFX(MotorIOTalonFXConfig config) {
        super(config);
    }

}
