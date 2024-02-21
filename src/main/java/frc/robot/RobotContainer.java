// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.mechlib.commands.SwerveTeleopDriveCommand;
import com.mechlib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final SendableChooser<Command> routineChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> quasistaticChooser = new SendableChooser<>();
  public final Swerve swerve = new Swerve();
  public final Arm arm = new Arm();
  public final Gerald gerald = new Gerald();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();

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
    // left trigger: intake
    xboxController.leftTrigger().onTrue(
            new InstantCommand(gerald::intake)
    );
    xboxController.leftBumper().onTrue(
            new InstantCommand(gerald::stopIntake)
    );

    // right trigger: shoot
    xboxController.rightTrigger().onTrue(
            new InstantCommand(gerald::shoot)
    );
    xboxController.rightBumper().onTrue(
            new InstantCommand(gerald::stopShooter)
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


  public Command getAutonomousCommand() { return new InstantCommand(); }
}
