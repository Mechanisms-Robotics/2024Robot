package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * Undisables the arm
 */
public class UnDisable extends InstantCommand {
    /**
     * Calls arm.unDisable
     *
     * @param arm instance of arm
     */
    public UnDisable(Arm arm) {
        super(arm::unDisable);
    }
}
