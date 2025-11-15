package frc.robot.Bases;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputIOReal implements DigitalInputIO {

    //NOTE are these for beambreaks? this is least important
    //Look at 1678's code for BeamBreaks. it looks like they generalized all external robot triggers as "BeamBreaks", not necessarily just beambreaks
    //like they have a "beambreak" for motor current spikes to detect if they intaked an algae, etc. 
    //So I think it's a better idea to call it something like "RobotTriggers". 
    //They also have this thing called a BeamBreakIOSim which extends the BeamBreakIO and looks like it's just used as a wrapper to trigger any boolean conditions,
    //They they use the Sim version for both sim and irl, which I don't think is necessary. Try to just remove the Sim version and only use BeamBreakIO.
    
    DigitalInput DI1 = null;
    DigitalInput DI2 = null;

    public DigitalInputIOReal(DigitalInput input) {
        DI1 = input;
    }

    public DigitalInputIOReal(DigitalInput input1, DigitalInput input2) {
        DI1 = input1;
        DI2 = input2;
    }

    public void updateInputs(DigitalIOInputs inputs) {
        inputs.DigitalInput1 = DI1.get();
        inputs.DigitalInput2 = DI2.get();
    }

    public DigitalIOInputs getDigitalIOInputs() {
        return new DigitalInputIO.DigitalIOInputs();
    }
}
