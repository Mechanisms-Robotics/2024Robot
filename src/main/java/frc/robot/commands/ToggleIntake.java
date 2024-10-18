package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

/** Toggles intake spinning */
public class ToggleIntake extends SequentialCommandGroup {
    /**
     * Calls gerald.toggleIntake
     *
     * @param gerald instance of gerald
     */
    public ToggleIntake(Gerald gerald) {
        addCommands(new InstantCommand(gerald::toggleIntake));
    }

    public ToggleIntake(Gerald gerald, ArmWrist armWrist) {
        addCommands(new InstantCommand(gerald::toggleIntake),
                new WaitUntilCommand(() -> gerald.noteDetected() || !armWrist.isIntaking()),
                new StowPosition(armWrist).onlyIf(armWrist::isIntaking),
                new PrintCommand("chinga sodhoi")
        );
    }
}
