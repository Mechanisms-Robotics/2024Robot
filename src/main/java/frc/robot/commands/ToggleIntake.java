package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/**
 * Toggles intake spinning
 */
public class ToggleIntake extends InstantCommand {
    /**
     * Calls gerald.toggleIntake
     *
     * @param gerald instance of gerald
     */
    public ToggleIntake(Gerald gerald) {
        super(gerald::toggleIntake);
    }
}
