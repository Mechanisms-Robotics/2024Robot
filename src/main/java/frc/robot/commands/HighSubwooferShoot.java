package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Pivots the armWrist ot the subwoofer high shoot angle
 */
public class HighSubwooferShoot extends InstantCommand {
    /**
     * Calls subwooferShoot in high mode to pivot the arm.
     *
     * @param armWrist instance of armWrist
     */
    public HighSubwooferShoot(ArmWrist armWrist) {
        super(() -> armWrist.subwooferShoot(false));
    }
}
