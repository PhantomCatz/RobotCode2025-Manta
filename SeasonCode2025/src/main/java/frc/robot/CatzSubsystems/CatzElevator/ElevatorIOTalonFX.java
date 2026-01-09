package frc.robot.CatzSubsystems.CatzElevator;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ElevatorIOTalonFX extends GenericTalonFXIOReal implements ElevatorIO {

    public ElevatorIOTalonFX(MotorIOTalonFXConfig config) {
        super(config);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        super.updateInputs(inputs);
    }

}
