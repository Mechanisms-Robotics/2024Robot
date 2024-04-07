package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/** Pivots the arm and wrist to the stow position */
public class StowPosition extends InstantCommand {
    /**
     * Calls armWrist.stow to pivot the arm and the wrist to the stow position
     *
     * @param armWrist instance of armWrist
     */
    public StowPosition(ArmWrist armWrist) {
        super(armWrist::stow);
    }
}
