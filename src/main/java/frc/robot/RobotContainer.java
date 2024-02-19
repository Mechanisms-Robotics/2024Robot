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
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final SendableChooser<Command> routineChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> quasistaticChooser = new SendableChooser<>();
  public final Swerve swerve = new Swerve();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();


    routineChooser.addOption(
            "Translational Quasistatic SysID",
            new InstantCommand(() -> swerve.setRoutine(SwerveDrive.SysIDRoutineType.Translational))
    );

    routineChooser.addOption(
            "Rotational Quasistatic SysID",
            new InstantCommand(() -> swerve.setRoutine(SwerveDrive.SysIDRoutineType.Rotational))
    );

    routineChooser.addOption(
            "Steer Quasistatic SysID",
            new InstantCommand(() -> swerve.setRoutine(SwerveDrive.SysIDRoutineType.Steer))
    );

    quasistaticChooser.addOption("Quasistatic", true);
    quasistaticChooser.addOption("Dynamic", false);

    SmartDashboard.putData("SysID Routine Chooser", routineChooser);
    SmartDashboard.putData("SysID Routine Type", quasistaticChooser);

    SignalLogger.setPath("home.lvuser/logs/");
    SignalLogger.start();
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
    if (quasistaticChooser.getSelected()) {
      return new SequentialCommandGroup(
              swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
              swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
      );
    } else {
      return new SequentialCommandGroup(
              swerve.sysIdDynamic(SysIdRoutine.Direction.kForward),
              swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse)
      );
    }
  }
  
}
