// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzIntakeRollers.RollerSubsytem;
import frc.robot.CatzSubsystems.CatzWrist.CatzWrist;

public class RobotContainer {

    public static final RobotContainer Instance = new RobotContainer();

    private final CommandXboxController xboxDrv = new CommandXboxController(0);

    // private final CatzSuperstructure superstructure = CatzSuperstructure.Instance; // Just to make sure it gets constructed

    private final RollerSubsytem rollers = RollerSubsytem.Instance; // TODO Rename this to CatzRollerSubsystem

    // private final RollerSubsytem CatzRollerSubsystem = RollerSubsytem.Instance; // TODO Rename this to CatzRollerSubsystem

    // private final CatzElevator CatzElevatorSubsystem = CatzElevator.Instance;

    // private final CatzArm CatzArmSubsystem = CatzArm.Instance;

    // private final CatzWrist CatzWristSubsystem = CatzWrist.Instance;

    // private final CatzDrivetrain CatzDriveSubsystem = CatzDrivetrain.Instance;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //xboxDrv.a().onTrue(CatzArm.Instance.armUp().alongWith(CatzWrist.Instance.extendWrist()).alongWith(CatzElevator.Instance.Elevator_L4()));
        //xboxDrv.b().onTrue(CatzArm.Instance.armStow().alongWith(CatzWrist.Instance.Wrist_Home()).alongWith(CatzElevator.Instance.Elevator_Stow()));
        xboxDrv.b().onTrue(new RunCommand(() -> {rollers.setSpeed(0.3);}));

        xboxDrv.x().onTrue(CatzWrist.Instance.extendWrist());
        xboxDrv.y().onTrue(CatzWrist.Instance.Wrist_Home());

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
