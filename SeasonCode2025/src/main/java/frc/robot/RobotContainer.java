// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants;
import frc.robot.CatzSubsystems.CatzIntakeRollers.CatzRollers;

public class RobotContainer {

    public static final RobotContainer Instance = new RobotContainer();

    private final CommandXboxController xboxDrv = new CommandXboxController(0);

    // private final CatzSuperstructure superstructure = CatzSuperstructure.Instance; // Just to make sure it gets constructed

    private final CatzRollers rollers = CatzRollers.Instance;

    private final CatzElevator elevator = CatzElevator.Instance;

    // private final CatzArm CatzArmSubsystem = CatzArm.Instance;

    // private final CatzWrist CatzWristSubsystem = CatzWrist.Instance;

    // private final CatzDrivetrain CatzDriveSubsystem = CatzDrivetrain.Instance;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //xboxDrv.a().onTrue(CatzArm.Instance.armUp().alongWith(CatzWrist.Instance.extendWrist()).alongWith(CatzElevator.Instance.Elevator_L4()));
        //xboxDrv.b().onTrue(CatzArm.Instance.armStow().alongWith(CatzWrist.Instance.Wrist_Home()).alongWith(CatzElevator.Instance.Elevator_Stow()));
        //xboxDrv.b().onTrue(new RunCommand(() -> {rollers.setDutyCycle(2);;}));

        xboxDrv.x().onTrue(CatzElevator.Instance.setpointCommand(ElevatorConstants.L4_SCORE));
        xboxDrv.y().onTrue(CatzElevator.Instance.setpointCommand(ElevatorConstants.STOW));

    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            elevator.setpointCommand(ElevatorConstants.L4_SCORE),
            Commands.waitSeconds(3.0),
            elevator.setpointCommand(ElevatorConstants.STOW),
            Commands.print("Hi"),
            rollers.setDutyCycleCommand(0.6)
        );
    }
}
