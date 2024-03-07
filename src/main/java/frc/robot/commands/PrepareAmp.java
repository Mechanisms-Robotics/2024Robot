package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Gerald;

/**
 * Spins up the shooter for the amp by spinning them up in the same direction
 */
public class PrepareAmp extends InstantCommand {
    /**
     * Calls gerald.prepareAmp, spins the shooter motors in the same direction for amping
     *
     * @param gerald instance of gerald
     */
    public PrepareAmp(Gerald gerald) {
        super(gerald::prepareAmp);
    }
}
