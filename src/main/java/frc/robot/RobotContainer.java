// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.mechlib.commands.SwerveTeleopDriveCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  public final Swerve swerve = new Swerve();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();
//    m_chooser.setDefaultOption("TopLeave");
  }

  public void configureBindings() {
    xboxController.a().onTrue( // a is uh x cuh
            new InstantCommand(swerve::zeroGyro)
    );

    xboxController.b().onTrue(
            new InstantCommand(swerve::lock)
    );
    // Test implementation for aim at
    xboxController.rightTrigger().whileTrue(
            new FunctionalCommand(
                    () -> {},
                    () -> {
//                      swerve.aimAt(new Translation2d(17, 5.7), new Rotation2d());
                    },
                    (interupted) -> {},
                    () -> false
            )
    );
  }

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(new SwerveTeleopDriveCommand(
            swerve,
            () -> -xboxController.getLeftY(),
            () -> -xboxController.getLeftX(),
            xboxController::getRightX,
            0.1,
            4.5,
            5,
            Math.PI,
            2*Math.PI,
            true
    ));
  }


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
  
}
