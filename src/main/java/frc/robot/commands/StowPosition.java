package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Wrist;

public class StowPosition extends InstantCommand {
    public StowPosition(ArmWrist armWrist) {
        super(armWrist::stow);
    }
}
