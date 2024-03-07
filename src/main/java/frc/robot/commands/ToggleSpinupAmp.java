package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/**
 * Toggles amp spinup
 */
public class ToggleSpinupAmp extends InstantCommand {
    /**
     * calls gerald.toggleSpinupAmp
     *
     * @param gerald instance of gerald
     */
    public ToggleSpinupAmp(Gerald gerald) {
        super(gerald::toggleSpinupAmp);
    }
}
