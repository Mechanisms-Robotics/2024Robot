package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/**
 * Toggle shoot spinup
 */
public class ToggleSpinupShoot extends InstantCommand {
    /**
     * Calls gerald.toggleSpinupShoot
     *
     * @param gerald instance of gerald
     */
    public ToggleSpinupShoot(Gerald gerald) {
        super(gerald::toggleSpinupShoot);
    }
}
