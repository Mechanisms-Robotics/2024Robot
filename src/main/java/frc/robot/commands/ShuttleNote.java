package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class ShuttleNote extends InstantCommand {
    public ShuttleNote(ArmWrist armWrist) {
        super(armWrist::shuttle);
    }
}
