package frc.robot.CatzSubsystems.CatzElevator;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ElevatorIOSim extends GenericIOSim<ElevatorIO.ElevatorIOInputs> implements ElevatorIO {

    public ElevatorIOSim(double gearRatio, Gains gains) {
        super(gearRatio, gains);
    }
}
