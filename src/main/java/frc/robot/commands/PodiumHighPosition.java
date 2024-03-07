package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Pivots to the podium high position
 */
public class PodiumHighPosition extends InstantCommand {
    /**
     * Calls armWrist.podiumShoot in high mode to pivot the arm and wrist to podium shoot high
     *
     * @param armWrist instance of armWrist
     */
    public PodiumHighPosition(ArmWrist armWrist) {
        super(() -> armWrist.podiumShoot(false));
    }
}
