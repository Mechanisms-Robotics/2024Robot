package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class PodiumLowPosition extends InstantCommand {
    public PodiumLowPosition(ArmWrist armWrist) {
        super(() -> armWrist.podiumShoot(true));
    }
}
