// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.mechlib.commands.SwerveTeleopDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private final Swerve swerve = new Swerve();

  public final Intake intake = new Intake();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();
  }

  public void configureBindings() {
    xboxController.a().onTrue(
            new InstantCommand(swerve::zeroGyro)
    );

    xboxController.b().onTrue(
            new InstantCommand(swerve::lock)
    );
    xboxController.x().onTrue(
            new InstantCommand(intake::deploy)
    );
    xboxController.y().onTrue(
            new InstantCommand(intake::retract)
    );
  }

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(new SwerveTeleopDriveCommand(
            swerve,
            () -> -xboxController.getLeftY(),
            () -> -xboxController.getLeftX(),
            () -> -xboxController.getRightX()
    ));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  
}
