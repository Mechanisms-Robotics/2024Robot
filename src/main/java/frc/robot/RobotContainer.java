// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.mechlib.commands.SwerveTeleopDriveCommand;
import com.mechlib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.TimedLeave;
import frc.robot.commands.autos.TimedShootLeave;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  private final SendableChooser<Command> routineChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> quasistaticChooser = new SendableChooser<>();
  public final Swerve swerve = new Swerve();
  public final Arm arm = new Arm();
  public final Gerald gerald = new Gerald();
  public final Wrist wrist = new Wrist();

  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxController2 = new CommandXboxController(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();
  }

  public void configureBindings() {
    xboxController.a().onTrue( // a is uh x cuh
            new InstantCommand(swerve::zeroGyro)
    );
    // b pressed: lock the swerve in x configuration
    xboxController.b().onTrue(
            new InstantCommand(swerve::lock)
    );

    // hold left trigger: intake
    xboxController.leftTrigger().whileTrue(new IntakeCommand(arm, wrist, gerald));

    // hold right trigger: shoot
    xboxController.rightTrigger().onTrue(
        new ParallelCommandGroup(
                  new InstantCommand(gerald::shoot),
                  new InstantCommand(arm::shoot),
                  new InstantCommand(wrist::shoot)
        )
    ).onFalse(new SequentialCommandGroup(
        new InstantCommand(gerald::feed),
        new WaitCommand(1.0),
        new InstantCommand(gerald::stopIntake),
        new InstantCommand(gerald::stopShooter),
        new InstantCommand(arm::stow),
        new InstantCommand(wrist::stow)
    ));
    // right bumper: amp
    xboxController.rightBumper().onTrue(
        new ParallelCommandGroup(
                  new InstantCommand(gerald::amp),
                  new InstantCommand(arm::shoot),
                  new InstantCommand(wrist::shoot)
        )
    ).onFalse(new SequentialCommandGroup(
        new InstantCommand(gerald::feed),
        new WaitCommand(1.0),
        new InstantCommand(gerald::stopIntake),
        new InstantCommand(gerald::stopShooter),
        new InstantCommand(arm::stow),
        new InstantCommand(wrist::stow)
    ));
    // up on d-pad: move arm and wrist to shoot/amp position
    xboxController.povUp().onTrue(
        new ParallelCommandGroup(
          new InstantCommand(arm::shoot),
          new InstantCommand(wrist::shoot)
        )
    );
    // right on d-pad: set arm and wrist to shoot/amp position
    xboxController.povRight().onTrue(
        new ParallelCommandGroup(
          new InstantCommand(arm::stow),
          new InstantCommand(wrist::stow)
        )
    );
    // down on d-pad: set arm and wrist to intake position
    xboxController.povDown().onTrue(
        new ParallelCommandGroup(
          new InstantCommand(arm::intake),
          new InstantCommand(wrist::intake)
        )
    );

    // secondary driver x: disable the arm (SAFETY)
    xboxController2.a().onTrue( // x on xbox controller
            new InstantCommand(arm::disable)
    );
  }

  private void configureDefaultCommands() {
    //Set the swerves default command to teleop drive
    swerve.setDefaultCommand(new SwerveTeleopDriveCommand(
            swerve,
            () -> -xboxController.getLeftY(),
            () -> -xboxController.getLeftX(),
            () -> -xboxController.getRightX(),
            0.1,
            4.5,
            5,
            Math.PI,
            2*Math.PI,
            true
    ));
  }

  public Command getAutonomousCommand() { return new TimedShootLeave(swerve, gerald, arm ); }
}
