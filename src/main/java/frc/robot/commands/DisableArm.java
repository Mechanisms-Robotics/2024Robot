package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * Disable the arm (safety)
 */
public class DisableArm extends InstantCommand {
    /**
     * Calls arm.disable
     *
     * @param arm instance of arm
     */
    public DisableArm(Arm arm) {
        super(arm::disable);
    }
}
