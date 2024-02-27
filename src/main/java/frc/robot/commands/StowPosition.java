package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class StowPosition extends ParallelCommandGroup {
    public StowPosition(Arm arm, Wrist wrist) {
        addCommands(
                new InstantCommand(arm::stow),
                new InstantCommand(wrist::stow)
        );
    }
}
