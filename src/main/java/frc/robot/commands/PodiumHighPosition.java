package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class PodiumHighPosition extends InstantCommand {
    public PodiumHighPosition(ArmWrist armWrist) {
        super(() -> armWrist.podiumShoot(false));
    }
}
