package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class ShuttleNote extends ParallelCommandGroup {
    public ShuttleNote(ArmWrist armWrist, Gerald gerald) {
        addCommands(new InstantCommand(armWrist::shuttle), new InstantCommand(gerald::prepareShoot));
    }
}
