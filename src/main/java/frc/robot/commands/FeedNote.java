package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

public class FeedNote extends InstantCommand {
    public FeedNote(Gerald gerald) {
        super(gerald::feed);
    }
}
