package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class ShuttleNote extends InstantCommand {
    public ShuttleNote(ArmWrist armWrist) {
        super(armWrist::shuttle);
    }
}
