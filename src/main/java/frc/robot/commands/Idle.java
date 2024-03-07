package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/**
 * Idles gerald, spinning slowly.
 */
public class Idle extends InstantCommand {
    /**
     * Calls gerald.idle to slowly spin the shooter motors
     *
     * @param gerald instance of gerald
     */
    public Idle(Gerald gerald) {
        super(gerald::idle);
    }
}
