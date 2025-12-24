package frc.robot.CatzSubsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CatzSubsystems.CatzArm.CatzArm;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzIntakeRollers.CatzRollers;
import frc.robot.CatzSubsystems.CatzWrist.CatzWrist;
import frc.robot.Utilities.VirtualSubsystem;

public class CatzSuperstructure extends VirtualSubsystem{
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    private final SubystemVisualizer setpointVisualizer = new SubystemVisualizer("Setpoint");

    public static final CatzElevator elevator = CatzElevator.Instance;
    public static final CatzWrist wrist = CatzWrist.Instance;
    public static final CatzArm arm = CatzArm.Instance;

    private CatzSuperstructure() {
        // TODO Auto-generated constructor stub
    }

    @Override
    public void periodic() {

        setpointVisualizer.update(elevator.getPosition(),
                                  Rotation2d.fromDegrees(0),
                                  Rotation2d.fromDegrees(wrist.getWristPos()));
    }

    public Command setSpeed() {
        return new RunCommand(() -> {CatzRollers.Instance.setDutyCycle(0);});
    }
}
