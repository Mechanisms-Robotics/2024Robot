package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Pivots to the podium low position
 */
public class PodiumLowPosition extends InstantCommand {
    /**
     * Calls armWrist.podiumShoot in low mode
     *
     * @param armWrist instance of armWrist
     */
    public PodiumLowPosition(ArmWrist armWrist) {
        super(() -> armWrist.podiumShoot(true));
    }
}
