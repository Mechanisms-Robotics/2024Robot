package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Disable the arm (safety)
 */
public class DisableArm extends InstantCommand {
    public DisableArm(Arm arm) {
        super(arm::disable);
    }
}
