package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Sets the arm and the wrist to the subwoofer shoot high angle
 */
public class SubwooferHighPosition extends InstantCommand {
    /**
     * Calls armWrist.subwooferShoot in high mode to pivot the arm and the wrist
     *
     * @param armWrist intance of armWrist
     */
    public SubwooferHighPosition(ArmWrist armWrist) {
        super(() -> armWrist.subwooferShoot(false));
    }
}
