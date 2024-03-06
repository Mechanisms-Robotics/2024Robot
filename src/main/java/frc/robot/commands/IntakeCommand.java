package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand(Gerald gerald) {
        addCommands(new InstantCommand(gerald::intake));
    }
    public IntakeCommand(ArmWrist armWrist, Gerald gerald) {
        addCommands(new IntakePosition(armWrist), new IntakeCommand(gerald));
    }
}