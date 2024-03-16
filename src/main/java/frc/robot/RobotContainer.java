// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.mechlib.commands.SwerveTeleopDriveCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.autos.AutoAimShootIntake;
import frc.robot.commands.autos.AutoAimShootStow;
import frc.robot.commands.autos.TimedLeave;
import frc.robot.commands.autos.TimedShootLeave;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final SendableChooser<Command> routineChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> quasistaticChooser = new SendableChooser<>();
  public final Swerve swerve = new Swerve();
  public final Gerald gerald = new Gerald();
  public final Arm arm = new Arm();
  public final Wrist wrist = new Wrist(swerve::getPitch, swerve::getRoll);
  public final ArmWrist armWrist = new ArmWrist(arm, wrist);
  public final LimeLight limeLight = new LimeLight();
  public final LED led = new LED();

  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxController2 = new CommandXboxController(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("intake", new IntakeCommand(armWrist, gerald));
    NamedCommands.registerCommand("spinup", new PrepareShoot(gerald));
    NamedCommands.registerCommand("spindown", new Idle(gerald));
    NamedCommands.registerCommand("aim", new DriveWhileAim(swerve, limeLight, armWrist).withTimeout(1.));
    NamedCommands.registerCommand("shoot", new FeedNote(gerald));
    NamedCommands.registerCommand("stow", new StowPosition(armWrist));
    NamedCommands.registerCommand("aimSubwooferHigh", new SubwooferHighPosition(armWrist));
    NamedCommands.registerCommand("aimShootStow", new AutoAimShootStow(armWrist, gerald, swerve, limeLight));
    NamedCommands.registerCommand("aimShootIntake", new AutoAimShootIntake(armWrist, gerald, swerve, limeLight));
    NamedCommands.registerCommand("aimC", new Aim(arm, wrist, 94, 115));

    NamedCommands.registerCommand("aimSubwooferLowShootIntake",
            new AutoAimShootIntake(armWrist, gerald, true, true));

    NamedCommands.registerCommand("aimSubwooferHighShootIntake",
            new AutoAimShootIntake(armWrist, gerald, true, false));

    NamedCommands.registerCommand("aimPodiumLowShootIntake",
            new AutoAimShootIntake(armWrist, gerald, false, true));

    NamedCommands.registerCommand("aimPodiumHighShootIntake",
            new AutoAimShootIntake(armWrist, gerald, false, false));

//    m_chooser.addOption("YeetRight", new PathPlannerAuto("YeetRight"));
//    m_chooser.addOption("YeetLeft", new PathPlannerAuto("YeetRight"));
//    m_chooser.addOption("TimedLeave", new TimedLeave(swerve));
    m_chooser.addOption("TimedShootLeave", new TimedShootLeave(swerve, gerald, armWrist));
    // ----------------2Note----------------
    m_chooser.addOption("SubRNoteR2Note", new PathPlannerAuto("SubRNoteR2Note"));
    m_chooser.setDefaultOption("SubCNoteC2Note", new PathPlannerAuto("SubCNoteC2Note"));
    m_chooser.addOption("SubLNoteL2Note", new PathPlannerAuto("SubLNoteL2Note"));
    // ----------------3Note----------------
//    m_chooser.addOption("SubCNoteCR3Note", new PathPlannerAuto("SubCNoteCR3Note"));
//    m_chooser.addOption("SubLNoteLC3Note", new PathPlannerAuto("SubLNoteLC3Note"));
    // ----------------4Note----------------
//    m_chooser.addOption("SubLNoteLCR4Note", new PathPlannerAuto("SubLNoteLCR4Note"));
    // ----------------Field----------------
//    m_chooser.addOption("SubRFieldRR1NoteGrab", new PathPlannerAuto("SubRFieldRR1NoteGrab"));
    m_chooser.addOption("TuningL", new PathPlannerAuto("TuningL"));
    m_chooser.addOption("TuningC", new PathPlannerAuto("TuningC"));

    SmartDashboard.putData("Auto Chooser", m_chooser);
    configureDefaultCommands();
  }

  public void configureBindings() {
    //////////////////
    //Primary Driver//
    //////////////////

    // -----------------Left-----------------
    xboxController.leftTrigger().onTrue(
            new ToggleIntake(gerald)
    );

    xboxController.leftBumper().onTrue(
            new ToggleSpinupAmp(gerald)
    );

    xboxController.leftStick().onTrue(
            new ZeroGyro(swerve)
    );

    // -----------------Right-----------------
    xboxController.rightBumper().onTrue(
            new ToggleSpinupShoot(gerald)
    );

    xboxController.rightTrigger().whileTrue(
            new FeedNote(gerald)
    );

    xboxController.rightStick().onTrue(
            new OuttakeCommand(gerald)
    ).onFalse(new Idle(gerald));

    xboxController.x().onTrue(
            new SubwooferHighPosition(armWrist)
    );

    xboxController.y().onTrue(
            new PodiumHighPosition(armWrist)
    );

    xboxController.a().onTrue(
            new IntakePosition(armWrist)
    );
    xboxController.b().onTrue(
            new SubwooferLowPosition(armWrist)
    );

    ////////////////////
    //Secondary Driver//
    ////////////////////

    // -----------------Left-----------------
    xboxController2.leftTrigger().onTrue(
            new SubwooferLowPosition(armWrist)
    );

    xboxController2.leftBumper().onTrue(
            new SubwooferHighPosition(armWrist)
    );

    xboxController2.leftStick().whileTrue(
            new Home(arm)
    );
    // -----------------Right-----------------
    xboxController2.rightTrigger().onTrue(
            new PodiumLowPosition(armWrist)
    );

    xboxController2.rightBumper().onTrue(
            new PodiumHighPosition(armWrist)
    );

    xboxController2.rightStick().whileTrue(
            new Climb(armWrist)
    );

    xboxController2.x().whileTrue( // square on ps4
            new DriveWhileAim(swerve, limeLight, armWrist,
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
    xboxController2.b().onTrue(
            new ShuttleNote(armWrist)
    );

    xboxController2.povDown().onTrue(
            new StowPosition(armWrist)
    );
    xboxController2.povUp().whileTrue(
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
            4,
            4.5,
            Math.PI*1,
            Math.PI*2,
            true
    ));
    led.setDefaultCommand(new LEDCommand(led, gerald, limeLight::getData, armWrist));
  }

  public Command getAutonomousCommand() { return m_chooser.getSelected(); }
}
