package frc.robot.CatzSubsystems.CatzElevator;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzElevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOTalonFX extends GenericTalonFXIOReal<ElevatorIO.ElevatorIOInputs> implements ElevatorIO {

    private final DigitalInput forwardLimitSwitch = new DigitalInput(0); // Replace 0 with actual channel


    public ElevatorIOTalonFX(MotorIOTalonFXConfig config) {
        super(config);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.forwardLimitSwitch = forwardLimitSwitch.get();


    }

}
