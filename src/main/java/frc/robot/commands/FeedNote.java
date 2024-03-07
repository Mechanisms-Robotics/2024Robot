package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/**
 * Runs the intake into the shooter.
 * Used for shooting, amping, and outtaking.
 */
public class FeedNote extends InstantCommand {
    /**
     * Calls the feed method of gerald
     *
     * @param gerald instance of the gerald subsystem
     */
    public FeedNote(Gerald gerald) {
        super(gerald::feed);
    }
}
