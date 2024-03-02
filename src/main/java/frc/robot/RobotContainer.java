// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.mechlib.commands.SwerveTeleopDriveCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.autos.TimedShootLeave;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final SendableChooser<Command> routineChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> quasistaticChooser = new SendableChooser<>();
  public final Swerve swerve = new Swerve();
  public final Arm arm = new Arm();
  public final Gerald gerald = new Gerald();
  public final Wrist wrist = new Wrist();
  public final ArmWrist armWrist = new ArmWrist();
  public final LimeLight limeLight = new LimeLight();

  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxController2 = new CommandXboxController(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    configureDefaultCommands();
  }

  public void configureBindings() {
    //////////////////
    //Primary Driver//
    //////////////////
    xboxController.leftStick().onTrue(
            new ZeroGyro(swerve)
    );

    xboxController.leftTrigger().onTrue(
            new ToggleIntake(gerald)
    );

    xboxController.leftBumper().onTrue(
            new ToggleSpinupAmp(gerald)
    );

    xboxController.rightBumper().onTrue(
            new ToggleSpinupShoot(gerald)
    );

    xboxController.rightTrigger().onTrue(
            new FeedNote(gerald)
    );

    xboxController.y().onTrue(
            new PodiumHighPosition(armWrist)
    );

    xboxController.b().onTrue(
            new SubwooferLowPosition(armWrist)
    );

    xboxController.x().onTrue(
            new SubwooferHighPosition(armWrist)
    );

    xboxController.a().onTrue(
            new IntakePosition(armWrist)
    );

    ////////////////////
    //Secondary Driver//
    ////////////////////

    xboxController2.leftTrigger().onTrue(
            new SubwooferLowPosition(armWrist)
    );

    xboxController2.leftBumper().onTrue(
            new SubwooferHighPosition(armWrist)
    );

    xboxController2.rightTrigger().onTrue(
            new PodiumLowPosition(armWrist)
    );

    xboxController2.rightBumper().onTrue(
            new PodiumHighPosition(armWrist)
    );

    xboxController2.rightStick().whileTrue(
            new DriveWhileAim(swerve, limeLight,
                    () -> -xboxController.getLeftY(),
                    () -> -xboxController.getLeftX(),
                    () -> -xboxController.getRightX())
    );

    xboxController2.y().onTrue(
            new DisableArm(arm)
    );

    xboxController2.a().onTrue(
            new IntakePosition(armWrist)
    );

    xboxController2.povUp().onTrue(
            new StowPosition(armWrist)
    );
    xboxController.povDown().whileTrue(
            new Savery(armWrist)
    );
  }


  private void configureDefaultCommands() {
    // Set the swerves default command to teleop drive
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

  public Command getAutonomousCommand() { return new TimedShootLeave(swerve, gerald, armWrist); }
}
