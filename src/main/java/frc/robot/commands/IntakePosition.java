package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Sets the arm and the wrist to the intake position.
 */
public class IntakePosition extends InstantCommand {
    /**
     * Calls armWrist.intake for setting the arm and the wrist to the intake position.
     *
     * @param armWrist
     */
    public IntakePosition(ArmWrist armWrist) {
        super(armWrist::intake);
    }
}
